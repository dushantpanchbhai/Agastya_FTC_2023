package org.firstinspires.ftc.teamcode.drive.opmode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class vision extends  OpenCvPipeline{
//    private int width;
//    public vision(int width) {
//        this.width = width;
//    }

    Mat mat=new Mat();

    Mat newmat=new Mat();
    Mat og=new Mat();

    @Override
    public  Mat processFrame(Mat input){
        // thresholding
        Imgproc.cvtColor(input,og,Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_BGR2HSV);
        Scalar l_b=new Scalar(9,62,79);
        Scalar u_b=new Scalar(83,255,255);


        Core.inRange(mat,l_b,u_b,newmat);
//        Core.bitwise_and(mask,mat,mat);
        // blurring image

//        Imgproc.GaussianBlur(mat,mat,new Size(5,5),5);

        // removing noise
//        kernel.
        Imgproc.morphologyEx(newmat,newmat,Imgproc.MORPH_OPEN,Mat.ones(new Size(5,5), CvType.CV_32F));
        Imgproc.erode(newmat,newmat,Mat.ones(new Size(5,5), CvType.CV_32F));




        // finding contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(newmat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Adding top3 biggest contours
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        int num_detections=3;
        Rect[] boundRect = new Rect[contours.size()];
        int start=0;
        for (int i = 0; (i < contours.size()) && (start<num_detections); i++) {

            double area=Imgproc.contourArea(new MatOfPoint2f(contours.get(i).toArray()));
            if(area>200){
                contoursPoly[start] = new MatOfPoint2f(contours.get(i).toArray());
                boundRect[start] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[start].toArray()));
                start++;

            }

//
        }


        // drawing contours on mat

//        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_HSV2RGBA);
        for (int i = 0; i != num_detections; i++) {
            try {

                Imgproc.rectangle(og, boundRect[i], new Scalar(0.5, 76.9, 89.8));
//                Imgproc.rectangle(mat, boundRect[i], new Scalar(255, 0, 0));
            }
            catch(Exception e){
                break;
            }

        }




        return og;
    }
}
