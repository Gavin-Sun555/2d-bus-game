package game

import "reflect"

type snapshotCopy func(dst, src reflect.Value)

type snapshotCopyPlan struct {
	copy     snapshotCopy
	building bool
}

var copySaveSnapshot = planSnapshotCopy(reflect.TypeFor[gameSave](), make(map[reflect.Type]*snapshotCopyPlan))

// Plan the copy once from the save schema. Only exported JSON fields can be
// read by the encoder. Plain-value slices (road geometry, passenger queues)
// can be copied in bulk, while nested references need their own storage.
func planSnapshotCopy(t reflect.Type, plans map[reflect.Type]*snapshotCopyPlan) snapshotCopy {
	if plan, ok := plans[t]; ok {
		if plan.building {
			return func(dst, src reflect.Value) { copySnapshotValue(plan.copy, dst, src) }
		}
		return plan.copy
	}
	var plan snapshotCopy
	record := &snapshotCopyPlan{building: true}
	plans[t] = record
	switch t.Kind() {
	case reflect.Struct:
		type fieldCopy struct {
			index int
			copy  snapshotCopy
		}
		var fields []fieldCopy
		for i := 0; i < t.NumField(); i++ {
			field := t.Field(i)
			if !field.IsExported() || field.Tag.Get("json") == "-" {
				continue
			}
			if copy := planSnapshotCopy(field.Type, plans); copy != nil {
				fields = append(fields, fieldCopy{i, copy})
			}
		}
		if len(fields) != 0 {
			plan = func(dst, src reflect.Value) {
				dst.Set(src)
				for _, field := range fields {
					field.copy(dst.Field(field.index), src.Field(field.index))
				}
			}
		}
	case reflect.Pointer:
		copy := planSnapshotCopy(t.Elem(), plans)
		plan = func(dst, src reflect.Value) {
			if src.IsNil() {
				dst.SetZero()
				return
			}
			p := reflect.New(t.Elem())
			copySnapshotValue(copy, p.Elem(), src.Elem())
			dst.Set(p)
		}
	case reflect.Slice:
		copy := planSnapshotCopy(t.Elem(), plans)
		plan = func(dst, src reflect.Value) {
			if src.IsNil() {
				dst.SetZero()
				return
			}
			list := reflect.MakeSlice(t, src.Len(), src.Len())
			if copy == nil {
				reflect.Copy(list, src)
			} else {
				for i := 0; i < src.Len(); i++ {
					copy(list.Index(i), src.Index(i))
				}
			}
			dst.Set(list)
		}
	case reflect.Array:
		if copy := planSnapshotCopy(t.Elem(), plans); copy != nil {
			plan = func(dst, src reflect.Value) {
				for i := 0; i < src.Len(); i++ {
					copy(dst.Index(i), src.Index(i))
				}
			}
		}
	case reflect.Map:
		copy := planSnapshotCopy(t.Elem(), plans)
		plan = func(dst, src reflect.Value) {
			if src.IsNil() {
				dst.SetZero()
				return
			}
			dict := reflect.MakeMapWithSize(t, src.Len())
			item := reflect.New(t.Elem()).Elem()
			iter := src.MapRange()
			for iter.Next() {
				copySnapshotValue(copy, item, iter.Value())
				dict.SetMapIndex(iter.Key(), item)
			}
			dst.Set(dict)
		}
	case reflect.Interface:
		// Current saves use concrete data types. Require new interface fields
		// to define their snapshot ownership before the schema can use them.
		panic("autosave snapshot requires a concrete field type: " + t.String())
	}
	record.copy, record.building = plan, false
	return plan
}

func copySnapshotValue(copy snapshotCopy, dst, src reflect.Value) {
	if copy == nil {
		dst.Set(src)
		return
	}
	copy(dst, src)
}

func detachedSaveSnapshot(snapshot gameSave) gameSave {
	var detached gameSave
	copySaveSnapshot(reflect.ValueOf(&detached).Elem(), reflect.ValueOf(&snapshot).Elem())
	return detached
}
