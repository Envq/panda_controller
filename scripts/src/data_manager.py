#!/usr/bin/env python3

# data manager
import yaml
from yaml.scanner import ScannerError
import sys, os


# custom
from colors import print_col

# PATHS
DATA_FOLDER_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../data"))
MESH_FOLDER_PATH = DATA_FOLDER_PATH + "/meshes"
POSES_PATH = DATA_FOLDER_PATH + "/poses.yaml"
SCENES_PATH = DATA_FOLDER_PATH + "/scenes.yaml"

def getPathMesh(name):
    return MESH_FOLDER_PATH + "/" + name


# POSES--------------------------------------------------------------------
def load_pose(name):
    try:
        with open(POSES_PATH, 'r') as file:
            root = yaml.safe_load(file)
            if not root[name]:
                return None
            p = root[name]['position']
            o = root[name]['orientation']
            return [p['x'], p['y'], p['z'], o['x'], o['y'], o['z'], o['w']]

    except KeyError:
        print_col("[load_pose] Warning: {} not found".format(name), 'FG_YELLOW')
        return None
    except FileNotFoundError:
        print_col("[load_pose] Error: file not found ({})".format(POSES_PATH), 'FG_RED')
        return None
    except ScannerError:
        print_col("[load_pose] Error: badly formed file", 'FG_RED')   


def save_pose(pose, name):
    if isinstance(pose, list) and len(pose) == 7 and all(isinstance(e, int) or isinstance(e, float) for e in pose):
        try:
            # read
            with open(POSES_PATH, 'r') as file:
                root = yaml.safe_load(file)
                if not root: root = {} # initialize
                root.update({
                    name : {
                        'position' : {
                            'x' : pose[0],
                            'y' : pose[1],
                            'z' : pose[2]
                        },
                        'orientation' : {
                            'x' : pose[3],
                            'y' : pose[4],
                            'z' : pose[5],
                            'w' : pose[6]
                        }
                    }
                })
            # write
            with open(POSES_PATH, 'w') as file:
                yaml.safe_dump(root, file)

        except KeyError:
            print_col("[save_pose] Warning: {} not found".format(name), 'FG_YELLOW')
        except FileNotFoundError:
            print_col("[save_pose] Error: file not found ({})".format(POSES_PATH), 'FG_RED')
        except ScannerError:
            print_col("[save_pose] Error: badly formed file", 'FG_RED')   

    else:
         print_col("pose not valid", 'FG_RED')


# SCENES-------------------------------------------------------------------
def load_scene(name):
    try:
        with open(SCENES_PATH, 'r') as file:
            root = yaml.safe_load(file)
            return root[name]

    except KeyError:
        print_col("[load_scene] Warning: {} not found".format(name), 'FG_YELLOW')
        return None
    except FileNotFoundError:
        print_col("[load_scene] Error: file not found ({})".format(SCENES_PATH), 'FG_RED')
        return None
    except ScannerError:
        print_col("[load_scene] Error: badly formed file ({})".format(SCENES_PATH), 'FG_RED')   




# TEST---------------------------------------------------------------------
if __name__ == "__main__":
    print(load_pose("test_python"))
    save_pose([0.3,0.0,0.3, 0,0,0,10], "test_python")
    print(load_pose("test_python"))

    print(load_scene("pick_place"))