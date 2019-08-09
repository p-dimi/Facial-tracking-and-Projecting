# Projector_Camera_Calibrator
Allows you to project a light onto a detected face, using a camera and projector.

The main function of this application is to calibrate between a camera's capture image and a projector's projected image.

This is from an earlier project meant to track faces and shine a light onto them from a projector.

This code currently allows only for use of webcam as the camera, and both the projector and camera must face the same direction (the projector cannot be angled relative to the camera in any significant way).

I recently reworked the application to be a little bit more user friendly. This application is a standalone - you run it from command prompt or from git bash. After a series of input questions regarding the specs and locations of your camera and projector, the application will start and will shine a white light onto any face detected by the camera.

For it to work: You must connect a projector to your computer, and that projector must be projecting your computer screen's display.

The project uses Haar Cascade face detection.
