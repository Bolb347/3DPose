3D Object Pose Recognition using LINE-MOD-Pipeline and OpenCV's StereoBM solver. Install by following steps at https://github.com/aelmiger/LINE-MOD-Pipeline and then copying over the dylibs for wpiutils, wpi, and ntcore into the ext folder (replace the current ones if you are not on OSX)

This project is intended to provide a real-time alternativer to DNNs like FoundationPose.

To use, upload your CAD file (.ply) into the models folder and run the TemplateGenerator binary. Then change the model file in main.cpp in the line with detect(...) with your model's name. Change the cam1 and cam2 sources to the appropirate sources and update the camera intrinsics matrix.