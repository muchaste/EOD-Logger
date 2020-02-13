// function creates new files for data logging
/*----------------------------------------------------------------*/
void filestuff(String Date) {
  Serial.print(Date);

  fileNr++;
  String filename = Date + "_dma0_" + fileNr + ".bin";
  char fname[30];
  filename.toCharArray(fname, 30);
  Serial.print("filename: ");
  Serial.println(filename);

  if (!file.open(fname, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma0 file failed");
  }

  String filename1 = Date + "_dma1_" + fileNr + ".bin";

  filename1.toCharArray(fname, 30);
  Serial.print("filename: ");
  Serial.println(filename);
  if (!file1.open(fname, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma1 file failed");
  }
}



// function creates new file for correlation coefficient
/*----------------------------------------------------------------*/
void filestuffCorr(String Date) {
  fileNrCorr++;
  String filenameCorr = Date + "_Corr_" + fileNrCorr + ".bin";
  char fnameCorr[50];
  filenameCorr.toCharArray(fnameCorr, 50);
  Serial.print("filename: ");
  Serial.println(filenameCorr);
  if (!fileCorr.open(fnameCorr, O_RDWR | O_CREAT)) {
    sd.errorHalt("open Corr file failed");
  }
}
