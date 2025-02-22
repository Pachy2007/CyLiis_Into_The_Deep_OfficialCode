    package org.firstinspires.ftc.teamcode.Modules.Intake;

    import com.qualcomm.robotcore.hardware.ColorSensor;

    import org.firstinspires.ftc.teamcode.Robot.Hardware;

    public class SampleColor {


        public enum State{
            RED , BLUE , YELLOW;
        }
        public State state=State.BLUE;

        public float redError, yellowError , blueError;

        ColorSensor colorSensor;




        public float red=0 , blue=0 , green=0;

        public SampleColor()
        {
            colorSensor= Hardware.colorSensor;
        }




        private void updateColor()
        {
            red=colorSensor.red();
            green=colorSensor.green();
            blue=colorSensor.blue();

    //        red/=colorSensor.alpha();
      //      green/=colorSensor.alpha();
        //    blue/=colorSensor.alpha();


            //red=red*255;
            //green=green*255;
            //blue=blue*255;

        }


        public float distance(float r1 , float g1  , float b1 , float r2 , float g2 , float b2)
        {
            return (float)Math.sqrt( (r1-r2)*(r1-r2) + (b1-b2)*(b1-b2) + (g1-g2)*(g1-g2));
        }

        private void updateState()
        {
             redError=distance(red , green , blue , 255 ,0 , 0);
             yellowError=distance(red , green , blue , 255 , 255 , 0);
             blueError=distance(red , green , blue , 0 , 0 ,255);

            if(redError<= yellowError && redError<=blueError)state=State.RED;
            if(yellowError<= redError && yellowError<=blueError)state=State.YELLOW;
            if(blueError<= yellowError && blueError<=redError)state=State.BLUE;
        }


        public void update()
        {
            updateColor();
            updateState();
        }

    }
