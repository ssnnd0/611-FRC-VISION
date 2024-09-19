# Raspberry Pi Machine Learning Vision for FRC

## Inspiration 
Vision tracking for FRC (FIRST Robotics Competition).

## What does it do?
CustomVision.AI exports to Tensorflow and Onnx, which is used within my program to detect the object itself, then the Python program runs vision math on the supplied co-ordinates of any detected object.
to find out where it is in space. The program detects the object, and runs vision calculations to find distance to the object. It sends this information over the robot network using PyNetworkTables for additional processing on the roboRIO (e.g; pathfinding).

## Next Steps
We’d love to optimize our machine learning model by ditching CustomVision.AI and Tensorflow entirely and instead running the model on an OpenCV Dynamic Neural Network, and using all four cores on the Raspberry Pi. With this change, the vision program will be able to run in complete realtime—opening its use for on-the-fly pathfinding in the autonomous period and lining up with targets during the teleoperated period. Additionally, we’d love to extract 3D-world co-ordinates from the detected object using OpenCV, allowing us to go from driving straight with PID loops to generating paths on the fly with Pathfinder to get our robot to swerve directly to its target.

### Want to learn more?
[Google Doc using Tensorflow](https://docs.google.com/document/d/1xEkql4t2k2on5pWODVsJKmNB83CbAXsfhYoOYy8iIx8/edit?usp=sharing)

[Google Doc using ONNX)](https://docs.google.com/document/d/1wLhM5ahvdox7a_Fom5_leUtu3d5cdZXE6BZQBcUypsc/edit?usp=sharing)

## 🔒 Code License
This repository is licensed through the `The Unliscence`. More details are listed [here](https://github.com/ssnnd0/611-FRC-VISIOn/blob/main/LICENSE). 

```

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <https://unlicense.org>
```
