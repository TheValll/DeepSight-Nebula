import importlib.util
from pathlib import Path
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'prepare_ball_dataset.py'
SPEC = importlib.util.spec_from_file_location('prepare_dataset', SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PrepareTest(unittest.TestCase):
    def videos(self):
        result = []
        for environment in ['salon-jour', 'chambre-jour', 'chambre-nuit']:
            for color in ['white', 'orange']:
                for index in range(3):
                    result.append(f'20261010_120000_00000{index}_{environment}-scene{index}_{color}.avi')
        result += ['20261010_120000_000000_chambre-jour_none.avi',
                   '20261010_120000_000000_chambre-nuit_none.avi']
        return result

    def test_deterministic_whole_video_assignment(self):
        videos = self.videos()
        assignment = MODULE.plan_splits(videos, 42)
        self.assertEqual(assignment, MODULE.plan_splits(list(reversed(videos)), 42))
        self.assertEqual(set(assignment), set(videos))
        for split in ['val', 'test']:
            selected = [v for v in videos if assignment[v] == split]
            self.assertEqual(len(selected), 3)
            self.assertEqual(len({MODULE.video_info(v)['environment'] for v in selected}), 3)
            self.assertEqual({MODULE.video_info(v)['color'] for v in selected}, {'white', 'orange'})
        for video in videos:
            if MODULE.video_info(video)['color'] == 'none':
                self.assertEqual(assignment[video], 'train')
        for environment in ['salon-jour', 'chambre-jour', 'chambre-nuit']:
            self.assertEqual({MODULE.video_info(v)['color'] for v in videos
                              if assignment[v] == 'train'
                              and MODULE.video_info(v)['environment'] == environment
                              and MODULE.video_info(v)['color'] != 'none'}, {'white', 'orange'})

    def test_insufficient_groups_rejected(self):
        with self.assertRaises(ValueError):
            MODULE.plan_splits(self.videos()[:2], 42)

    def test_labels(self):
        self.assertEqual(MODULE.validate_label(''), 0)
        self.assertEqual(MODULE.validate_label('0 .5 .5 .2 .2\n0 .3 .3 .1 .1\n'), 2)
        for value in ['1 .5 .5 .2 .2', '0 .5 .5 nan .2', '0 .1 .1 1 1',
                      '0 .5 .5 0 .2', '0 .5 .5 .2']:
            with self.assertRaises(ValueError):
                MODULE.validate_label(value)

    def test_unknown_video_rejected(self):
        with self.assertRaises(ValueError):
            MODULE.video_info('unknown.avi')


if __name__ == '__main__':
    unittest.main()
