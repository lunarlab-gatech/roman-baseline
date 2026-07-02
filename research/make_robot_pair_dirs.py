import itertools
import os
import shutil

# Hardcoded parameters
BASE_DIR = os.path.expanduser('~/roman/results/hercules_V2.4.C')
SOURCE_MAP_DIR = os.path.join(BASE_DIR, 'latest', 'map')
ROBOT_NAMES = ['Husky1', 'Husky2', 'Drone1', 'Drone2']

if __name__ == '__main__':

    for robot_a, robot_b in itertools.combinations(ROBOT_NAMES, 2):
        pair_dir = os.path.join(BASE_DIR, f'{robot_a}_{robot_b}')
        map_dir = os.path.join(pair_dir, 'map')
        os.makedirs(map_dir, exist_ok=True)

        for robot in (robot_a, robot_b):
            for ext in ('.pkl', '.time.txt'):
                src = os.path.join(SOURCE_MAP_DIR, f'{robot}{ext}')
                dst = os.path.join(map_dir, f'{robot}{ext}')
                shutil.copy2(src, dst)

        print(f'Created {map_dir}')
