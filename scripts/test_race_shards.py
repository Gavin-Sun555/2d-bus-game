#!/usr/bin/env python3
import unittest
import tempfile
import subprocess
import json
import sys
from pathlib import Path
from run_race_shard import selected_tests


class PartitionTests(unittest.TestCase):
    def test_all_tests_exactly_once(self):
        names = [f'TestCase{i}' for i in range(1000)] + ['Example', 'FuzzSeed']
        for count in [1, 2, 8, 17]:
            assigned = [n for i in range(count) for n in selected_tests('pkg', names, i, count)]
            self.assertCountEqual(assigned, names)
            self.assertEqual(len(assigned), len(set(assigned)))

    def test_order_and_stability(self):
        names = [f'TestCase{i}' for i in range(100)]
        a = selected_tests('pkg', names, 2, 8)
        self.assertEqual(a, selected_tests('pkg', names, 2, 8))
        self.assertEqual(a, [n for n in names if n in a])
        self.assertEqual(selected_tests('pkg', [], 0, 8), [])

    def test_invalid_partition_rejected(self):
        for index, count in [(-1, 8), (8, 8), (0, 0)]:
            with self.assertRaises(ValueError):
                selected_tests('pkg', [], index, count)


class RunnerFixtureTests(unittest.TestCase):
    def test_real_go_suite_runs_exactly_once(self):
        script = Path(__file__).with_name('run_race_shard.py').resolve()
        with tempfile.TemporaryDirectory(prefix='race-shard-fixture-') as tmp:
            root = Path(tmp)
            (root / 'go.mod').write_text('module example.com/racefixture\n\ngo 1.27.1\n')
            (root / 'fixture_test.go').write_text('''package fixture
import ("testing"; "fmt")
func TestFirst(t *testing.T) {t.Run("child",func(t *testing.T){})}
func TestSecond(t *testing.T) {}
func FuzzSeed(f *testing.F) {f.Add(1);f.Fuzz(func(t *testing.T,n int){})}
func Example() {fmt.Println("ok")
// Output: ok
}
''')
            (root / 'empty').mkdir()
            (root / 'empty' / 'empty.go').write_text('package empty\n')
            roots = []
            empty_runs = 0
            for shard in range(2):
                output = root / f'out-{shard}'
                subprocess.run([sys.executable,str(script),'--index',str(shard),'--count','2','--output',str(output)],cwd=root,check=True,stdout=subprocess.PIPE,text=True)
                manifest = json.loads((output / 'manifest.json').read_text())
                empty_runs += sum(p['run'] for p in manifest['packages'] if not p['tests'])
                status = json.loads((output / 'status.json').read_text())
                self.assertFalse(any(status.values()))
                for log in output.glob('package-*.json'):
                    for line in log.read_text().splitlines():
                        event = json.loads(line)
                        name = event.get('Test','')
                        if event['Action']=='pass' and name and '/' not in name:
                            roots.append(name)
            self.assertCountEqual(roots,['TestFirst','TestSecond','FuzzSeed','Example'])
            self.assertEqual(empty_runs,1)


if __name__ == '__main__':
    unittest.main()
