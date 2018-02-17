package org.usfirst.frc.team4183.utils;

public enum JoystickScale
{
   LINEAR,
   SQUARE,
   CUBE,
   SINE;
   
   // Some utility functions to make later code more readable
   // All of these function preserve sign of input
   
   public double rescale(double x)
   {
      switch (this)
      {
      case SQUARE: // Decrease sensitivity
         x  = square(x);
         break;
      case CUBE: // Decrease sensitivity a little more
         x  = cube(x);
         break;
      case SINE: // Slightly decreased sensitivity and then approach linear
         x = sine(x);
         break;
      case LINEAR:
      default:
         break;
      }
      
      return x;
   }
   
   public double square(double x)
   {
      return Math.abs(x)*x;
   }
   
   public double cube(double x)
   {
      return x*x*x;
   }
   
   public double sine(double x)
   {
      x = Math.abs(x)*Math.sin(x*Math.PI/2.0);
      
      return x;
   }

}

