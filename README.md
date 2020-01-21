# LineFollowing
__author__ = "Yisak Kim"


## Normal mode
0. Place your robot on the 'Yellow line' of your gameboard.
2. Run 'Andycar3.py' on your Raspberry Pi using python3.
3. If you want to make it paused, press 'z', and press 'x' to resume.
    (command line will show you 'M-mode On/Off'.)
4. If you want to terminate this program, press 'q'.


## VideoCature mode
1. Uncomment below codes

- videoFile1 = './video001.avi'
- ourcc = cv2.VideoWriter_fourcc(*'DIVX')
- out = cv2.VideoWriter(videoFile1, fourcc, 9.0, (320,240))
- out.write(image)
- out.release()
- cv2.destroyAllWindows()

2. Set the path for the video file on " videoFile1 = './video001.avi' "
3. Run 'Andycar3.py' on your Raspberry Pi using python3, and the rest is the same as 'Normal mode' above.


## Object Detection
- Detect Green Sign, "def make_cropped()". See "ObjectDetection" repository.

