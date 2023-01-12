package org.firstinspires.ftc.teamcode.Vision;

import static java.lang.Integer.parseInt;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Main.Constants;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.classifier.Classifications;
import org.tensorflow.lite.task.vision.classifier.ImageClassifier;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;


/** Sources:
 *  - https://github.com/FIRST-Tech-Challenge/SkyStone/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptWebcam.java
 *  - https://www.tensorflow.org/lite/api_docs/java/org/tensorflow/lite/support/image/TensorImage
 *  - https://www.tensorflow.org/lite/inference_with_metadata/task_library/image_classifier#run_inference_in_java
 *  - https://github.com/tensorflow/tflite-support/blob/master/tensorflow_lite_support/java/src/java/org/tensorflow/lite/task/vision/classifier/ImageClassifier.java
 *  - https://www.tensorflow.org/lite/api_docs/java/org/tensorflow/lite/support/label/Category
 *  - https://www.tensorflow.org/lite/api_docs/java/org/tensorflow/lite/task/vision/classifier/ImageClassifier
 *
 *  recycle
 */
public class SignalClassifier {

    /** How long we are to wait to be granted permission to use the camera before giving up. Here,
     * we wait indefinitely */
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    /** Initialize Camera Variables */
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;

    /** Bitmap which stores the image from the camera */
    private Bitmap rawImage;

    /** A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}
     * if you're curious): no knowledge of multi-threading is needed here. */
    private Handler callbackHandler;

    private ImageClassifier model;

    private Telemetry telemetry;

    public static boolean isClassificationLoaded = false;

    //----------------------------------------------------------------------------------------------
    // SignalClassifier constructor
    //----------------------------------------------------------------------------------------------

    public SignalClassifier(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;

        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        loadModel();
        openCamera();
        startCamera();

        telemetry.addData("classification", "loaded");
        telemetry.update();
        isClassificationLoaded = true;
    }

    private void loadModel() {
        try {
            model = ImageClassifier.createFromFile(Constants.classification_model);
        } catch (IOException e) {
            telemetry.addData("error", e.toString());
            telemetry.update();
        }
    }

    public String classify() {
        // Convert bitmap to TensorImage
        TensorImage img = TensorImage.fromBitmap(rawImage);
        String Label = "";

        // Run inference
        List<Classifications> results = model.classify(img);

        // Find the most probable signal
        float max_score = 0;
        for (Category c : results.get(0).getCategories()) {
            if (c.getScore() > max_score) {
                max_score = c.getScore();
                Label = c.getLabel();
            }
        }

        return Label;
    }

//    /** Do something with the frame */
//    private void onNewFrame(Bitmap frame) {
//        classify(frame);
//        frame.recycle(); // not strictly necessary, but helpful
//    }

    //----------------------------------------------------------------------------------------------
    // Camera operations
    //----------------------------------------------------------------------------------------------

//    private void initializeFrameQueue(int capacity) {
//        /** The frame queue will automatically throw away bitmap frames if they are not processed
//         * quickly by the OpMode. This avoids a buildup of frames in memory */
//        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
//        frameQueue.setEvictAction(new Consumer<Bitmap>() {
//            @Override public void accept(Bitmap frame) {
//                // RobotLog.ii(TAG, "frame recycled w/o processing");
//                frame.recycle(); // not strictly necessary, but helpful
//            }
//        });
//    }

    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            error("camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();

        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /** Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                            new CameraCaptureSession.CaptureCallback() {
                                @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                    /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                     * for the duration of the callback. So we copy here manually. */
                                    rawImage = captureRequest.createEmptyBitmap();
                                    cameraFrame.copyToBitmap(rawImage);
                                }
                            },
                            Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
//                                    RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                }
                            })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException|RuntimeException e) {
//                        RobotLog.ee(TAG, e, "exception starting capture");
                        error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
//            RobotLog.ee(TAG, e, "exception starting camera");
            error("exception starting camera");
            synchronizer.finish(null);
        }

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }

    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    public void closeSignalClassifier() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utilities
    //----------------------------------------------------------------------------------------------

    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }
    private void error(String format, Object...args) {
        telemetry.log().add(format, args);
        telemetry.update();
    }

}
