# navigation
## Overview
navigationと連携して移動を命令するパッケージ。

## Usage

```
roslaunch navigation navigation.launch
```

## Node
**`name` navigation**

### Subscriber

* **`"/navigation/move_command"`** 移動座標を受け取る ( location/Location )


### msg
Location.msg

    string name
    float64 x
    float64 y
    float64 z

### srv
RegisterLocation.srv

    string name
    ---
RequestLocation.srv

    string name
    ---
    Location location
RequestCurrentLocation.srv

    ---
    Location location
RequestLocationList.srv

    ---
    Location[] locations