package traffic

import "reflect"

// exportedEqual compares the observable state of two values: exported fields,
// recursively. Unexported fields hold lazily built caches (for example a
// junction corridor's broad-phase groups) whose presence depends only on
// which queries happened to run, not on vehicle behavior.
func exportedEqual(a, b any) bool {
	return exportedValuesEqual(reflect.ValueOf(a), reflect.ValueOf(b))
}

func exportedValuesEqual(a, b reflect.Value) bool {
	if a.IsValid() != b.IsValid() {
		return false
	}
	if !a.IsValid() {
		return true
	}
	if a.Type() != b.Type() {
		return false
	}
	switch a.Kind() {
	case reflect.Pointer, reflect.Interface:
		if a.IsNil() || b.IsNil() {
			return a.IsNil() == b.IsNil()
		}
		return exportedValuesEqual(a.Elem(), b.Elem())
	case reflect.Struct:
		for i := 0; i < a.NumField(); i++ {
			if a.Type().Field(i).IsExported() && !exportedValuesEqual(a.Field(i), b.Field(i)) {
				return false
			}
		}
		return true
	case reflect.Slice, reflect.Array:
		if a.Kind() == reflect.Slice && a.IsNil() != b.IsNil() {
			return false
		}
		if a.Len() != b.Len() {
			return false
		}
		for i := 0; i < a.Len(); i++ {
			if !exportedValuesEqual(a.Index(i), b.Index(i)) {
				return false
			}
		}
		return true
	case reflect.Map:
		if a.IsNil() != b.IsNil() || a.Len() != b.Len() {
			return false
		}
		for _, key := range a.MapKeys() {
			if !exportedValuesEqual(a.MapIndex(key), b.MapIndex(key)) {
				return false
			}
		}
		return true
	case reflect.Float32, reflect.Float64:
		// Exact equality, treating NaN as equal to NaN.
		return a.Float() == b.Float() || (a.Float() != a.Float() && b.Float() != b.Float())
	default:
		return a.Equal(b)
	}
}
