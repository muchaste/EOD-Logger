// function returns Teensy 3.x's RTC
/*----------------------------------------------------------------*/
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}


// function to sync RTC Time by typing into the Serial Monitor input field: T 1580296420
// The number is the current unix timestamp: seconds since Jan 01 1970. (UTC)
// In https://www.unixtimestamp.com/index.php the date and time can be converted to unix
/*----------------------------------------------------------------*/
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; 			                  // default Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { 					                            // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}


// utility function for digital clock display: prints preceding colon and leading 0
/*----------------------------------------------------------------*/
void digitalClockDisplay() {
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}
