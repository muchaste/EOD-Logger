// function to calculate correlation between two float arrays
//
//
// input parameter:
//
// float    x[]                first array
// float    y[]		       second array
// int        n	               size of arrays, note that x and y must be of equal size
// float* rDest		       float pointer to location where correlation coefficient is stored



void corr(float x[], float y[] , int n, float* rDest) {

  float mx, my, sx, sy, sxy, denom;				  // init variables


  // calculate mean, std and covariance
  /*----------------------------------------------------------------*/
  arm_mean_f32(x, n, &mx);					                                     // calc mean
  arm_mean_f32(y, n, &my);
  // Serial.print("mean x: ");  Serial.println(mx);
  // Serial.print("mean y: ");  Serial.println(my);

  arm_std_f32(x, n, &sx);					                                       // calc std
  arm_std_f32(y, n, &sy);
  // Serial.print("std x: ");  Serial.println(sx);
  // Serial.print("std y: ");  Serial.println(sy);

  sxy = 0;							                                                 // calc covariance
  for (int i = 0; i < n; i++) {
    sxy += (x[i] - mx) * (y[i] - my);
  }

  //  calculate correlation coefficient
  /*----------------------------------------------------------------*/

  if (sxy > 4294967040) {					                                       // check for too large coariance --> in case this is not done one gets ovf values
    // Serial.print("too large nominator");
    *rDest = - 2; 					                                             // error val
  }
  else {
    denom = sx * sy * (n - 1);
    // Serial.print("denominator: ");
    // Serial.println(denom);

    if (denom == 0) {					                                           // check for zero denominator
      *rDest = 0;					                                               // zero covariance implies zero correlation coefficient
      Serial.print("inf");
    }
    else {
      *rDest = sxy / denom ;				                                      // store correlation coefficient
    }
  }
