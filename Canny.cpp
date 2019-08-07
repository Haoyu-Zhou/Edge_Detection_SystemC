/* source: http://marathon.csee.usf.edu/edge/edge_detection.html */
/* URL: ftp://figment.csee.usf.edu/pub/Edge_Comparison/source_code/canny.src */

/* ECPS 203 Assignment 4 solution */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "systemc.h"

#define VERBOSE 0

#define NOEDGE 255
#define POSSIBLE_EDGE 128
#define EDGE 0
#define BOOSTBLURFACTOR 90.0
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define SIGMA 0.6
#define TLOW  0.3
#define THIGH 0.8

#define COLS 2704
#define ROWS 1520
#define SIZE COLS*ROWS
#define VIDEONAME "Engineering"
#define IMG_IN    "video/" VIDEONAME "%03d.pgm"
#define IMG_OUT   VIDEONAME "%03d_edges.pgm"
#define IMG_NUM   30 /* number of images processed (1 or more) */
#define AVAIL_IMG 30 /* number of different image frames (1 or more) */

/* upper bound for the size of the gaussian kernel
 * SIGMA must be less than 4.0
 * check for 'windowsize' below
 */
#define WINSIZE 21

#define SET_STACK_SIZE set_stack_size(128 * 1024 * 1024)

 //IMAGE struct for ECPS 203 A5
 //copy the following definition of IMAGE to your source code
typedef struct Image_s
{
	unsigned char img[SIZE];
	Image_s(void) {
		for (int i = 0; i < SIZE; i++) {
			img[i] = 0;
		}
	}
	Image_s& operator=(const Image_s& copy) {
		for (int i = 0; i < SIZE; i++) {
			img[i] = copy.img[i];
		}
		return *this;
	}
	operator unsigned char*() {
		return img;
	}
	unsigned char& operator[](const int index) {
		return img[index];
	}
}IMAGE;


typedef struct SImage_s
{
	short int img[SIZE];
	SImage_s(void) {
		for (int i = 0; i < SIZE; i++) {
			img[i] = 0;
		}
	}
	SImage_s& operator=(const SImage_s& copy) {
		for (int i = 0; i < SIZE; i++) {
			img[i] = copy.img[i];
		}
		return *this;
	}
	operator short int*() {
		return img;
	}
	short int& operator[](const int index) {
		return img[index];
	}
}SIMAGE;


typedef struct Kernel_s
{
	float img[WINSIZE];
	Kernel_s(void) {
		for (int i = 0; i < WINSIZE; i++) {
			img[i] = 0;
		}
	}
	Kernel_s& operator=(const Kernel_s& copy) {
		for (int i = 0; i < WINSIZE; i++) {
			img[i] = copy.img[i];
		}
		return *this;
	}
	operator float*() {
		return img;
	}
	float& operator[](const int index) {
		return img[index];
	}
}KERNEL;


typedef struct Tempim_s
{
	float img[SIZE];
	Tempim_s(void) {
		for (int i = 0; i < SIZE; i++) {
			img[i] = 0;
		}
	}
	Tempim_s& operator=(const Tempim_s& copy) {
		for (int i = 0; i < SIZE; i++) {
			img[i] = copy.img[i];
		}
		return *this;
	}
	operator float*() {
		return img;
	}
	float& operator[](const int index) {
		return img[index];
	}
}TEMPIM;


//Stim Module
SC_MODULE(Stim)
{
	IMAGE img;
	sc_fifo_out<IMAGE> ImgOutPort;
	sc_fifo_out<sc_time> FTOutPort;
	sc_time startTime;

	int read_pgm_image(const char *infilename, unsigned char *image, int rows, int cols);
	void read_img_in();

	SC_CTOR(Stim) {
		SC_THREAD(read_img_in);
		set_stack_size(128 * 1024 * 1024);
	}
};


//Mon Module
SC_MODULE(Mon)
{
	IMAGE img;
	sc_fifo_in<IMAGE> ImgInPort;
	sc_fifo_in<sc_time> FTInPort;
	sc_time startTime;
	sc_time endTime;

	int write_pgm_image(const char *outfilename, unsigned char *image, int rows,
		int cols, const char *comment, int maxval);
	void print_img_out();

	SC_CTOR(Mon) {
		SC_THREAD(print_img_out);
		set_stack_size(128 * 1024 * 1024);
	}
};


//DataOut
SC_MODULE(DataOut)
{
	sc_fifo_in<IMAGE> ImgInPort;
	sc_fifo_out<IMAGE> ImgOutPort;
	IMAGE img;

	void transfer_out() {
		while (true) {
			ImgInPort.read(img);
			ImgOutPort.write(img);
		}
	}

	SC_CTOR(DataOut) {
		SC_THREAD(transfer_out);
		set_stack_size(128 * 1024 * 1024);
	}
};


//DataIn
SC_MODULE(DataIn)
{
	sc_fifo_in<IMAGE> ImgInPort;
	sc_fifo_out<IMAGE> ImgOutPort;
	IMAGE img;

	void transfer_in() {
		while (true) {
			ImgInPort.read(img);
			ImgOutPort.write(img);
		}
	}

	SC_CTOR(DataIn) {
		SC_THREAD(transfer_in);
		set_stack_size(128 * 1024 * 1024);
	}
};


//DUT
SC_MODULE(RECEIVER) {
	sc_fifo_in<IMAGE> ImgInPort;
	sc_fifo_out<IMAGE> ImgOutPort;
	IMAGE img;

	void receiver() {
		while (true) {
			ImgInPort.read(img);
			wait(0, SC_MS);
			ImgOutPort.write(img);
		}
	}

	SC_CTOR(RECEIVER) {
		SC_THREAD(receiver);
		SET_STACK_SIZE;
	}
};


SC_MODULE(GAUSSIAN_KERNEL) {
	sc_fifo_out<KERNEL> KernelToBlurXOutPort;
	sc_fifo_out<int> WsOutToBlurXPort;
	KERNEL kernel;
	int wsize;

	void make_gaussian_kernel(float sigma, float *kernel, int *windowsize);
	void start_make_gaussian_kernel();

	SC_CTOR(GAUSSIAN_KERNEL) {
		SC_THREAD(start_make_gaussian_kernel);
		SET_STACK_SIZE;
	}
};


SC_MODULE(BLURX) {
	sc_fifo_in<IMAGE> ImgInPort;
	sc_fifo_in<KERNEL> KernelInPort;
	sc_fifo_in<int> WsInPort;
	sc_fifo_out<TEMPIM> TempimOutPort;
	sc_fifo_out<KERNEL> KernelToBlurYOutPort;
	sc_fifo_out<int> WsOutToBlurYPort;
	IMAGE img;
	KERNEL kernel;
	int wsize;
	TEMPIM tempim;

	sc_event e1, e2, e3, e4, ef;

	void blurx1();
	void blurx2();
	void blurx3();
	void blurx4();
	void start_blurx();

	SC_CTOR(BLURX) {
		SC_THREAD(start_blurx);
		SET_STACK_SIZE;
		SC_THREAD(blurx1);
		SET_STACK_SIZE;
		SC_THREAD(blurx2);
		SET_STACK_SIZE;
		SC_THREAD(blurx3);
		SET_STACK_SIZE;
		SC_THREAD(blurx4);
		SET_STACK_SIZE;
	}
};


SC_MODULE(BLURY) {
	sc_fifo_in<TEMPIM> TempimInPort;
	sc_fifo_in<KERNEL> KernelInPort;
	sc_fifo_in<int> WsInPort;
	sc_fifo_out<SIMAGE> SdimOutPort;
	TEMPIM tempim;
	KERNEL kernel;
	int wsize;
	SIMAGE smoothedim;

	sc_event e1, e2, e3, e4, ef;

	void blury1();
	void blury2();
	void blury3();
	void blury4();
	void start_blury();

	SC_CTOR(BLURY) {
		SC_THREAD(start_blury);
		SET_STACK_SIZE;
		SC_THREAD(blury1);
		SET_STACK_SIZE;
		SC_THREAD(blury2);
		SET_STACK_SIZE;
		SC_THREAD(blury3);
		SET_STACK_SIZE;
		SC_THREAD(blury4);
		SET_STACK_SIZE;
	}
};


SC_MODULE(GAUSSIAN_SMOOTH) {
	sc_fifo_in<IMAGE> ImgInPort;
	sc_fifo_out<SIMAGE> SdimOutPort;

	sc_fifo<IMAGE> imageChannel;
	sc_fifo<KERNEL> kernelChannelX;
	sc_fifo<KERNEL> kernelChannelY;
	sc_fifo<int> wsizeChannelX;
	sc_fifo<int> wsizeChannelY;
	sc_fifo<TEMPIM> tempimChannel;

	RECEIVER receiverModule;
	GAUSSIAN_KERNEL kernelModule;
	BLURX blurxModule;
	BLURY bluryModule;

	void before_end_of_elaboration() {
		//port to port first
		receiverModule.ImgInPort.bind(ImgInPort);
		receiverModule.ImgOutPort.bind(imageChannel);

		kernelModule.KernelToBlurXOutPort.bind(kernelChannelX);
		kernelModule.WsOutToBlurXPort.bind(wsizeChannelX);


		blurxModule.ImgInPort.bind(imageChannel);
		blurxModule.KernelInPort.bind(kernelChannelX);
		blurxModule.WsInPort.bind(wsizeChannelX);
		blurxModule.TempimOutPort.bind(tempimChannel);
		blurxModule.KernelToBlurYOutPort.bind(kernelChannelY);
		blurxModule.WsOutToBlurYPort.bind(wsizeChannelY);

		bluryModule.TempimInPort.bind(tempimChannel);
		bluryModule.KernelInPort.bind(kernelChannelY);
		bluryModule.WsInPort.bind(wsizeChannelY);
		bluryModule.SdimOutPort.bind(SdimOutPort);
	}


	SC_CTOR(GAUSSIAN_SMOOTH) :
		kernelChannelX("kernelChannelX", 1), kernelChannelY("kernelChannelY", 1),
		wsizeChannelX("wsizeChannelX", 1), wsizeChannelY("wsizeChannelY", 1),
		imageChannel("imageChannel", 1), tempimChannel("tempimChannel", 1),
		receiverModule("receiverModule"), kernelModule("kernelModule"),
		blurxModule("blurxModule"), bluryModule("bluryModule") {
	}
};


SC_MODULE(DERIVATIVE_X_Y) {
	sc_fifo_in<SIMAGE> SdimInPort;
	sc_fifo_out<SIMAGE> DxToMagOutPort;
	sc_fifo_out<SIMAGE> DyToMagOutPort;
	SIMAGE sdim, dx, dy;

	void derrivative_x_y(short int *smoothedim, int rows, int cols,
		short int *delta_x, short int *delta_y);
	void start_derrivative_x_y();

	SC_CTOR(DERIVATIVE_X_Y) {
		SC_THREAD(start_derrivative_x_y);
		SET_STACK_SIZE;
	}
};


SC_MODULE(MAGNITUDE_X_Y) {
	sc_fifo_in<SIMAGE> DxInPort;
	sc_fifo_in<SIMAGE> DyInPort;
	sc_fifo_out<SIMAGE> MagToNonOutPort;
	sc_fifo_out<SIMAGE> DxToNonOutPort;
	sc_fifo_out<SIMAGE> DyToNonOutPort;
	SIMAGE dx, dy, mag;

	void magnitude_x_y(short int *delta_x, short int *delta_y, int rows, int cols,
		short int *magnitude);
	void start_magnitude_x_y();

	SC_CTOR(MAGNITUDE_X_Y) {
		SC_THREAD(start_magnitude_x_y);
		SET_STACK_SIZE;
	}
};


SC_MODULE(NON_MAX_SUPP) {
	sc_fifo_in<SIMAGE> MagInPort;
	sc_fifo_in<SIMAGE> DxInPort;
	sc_fifo_in<SIMAGE> DyInPort;
	sc_fifo_out<IMAGE> NmsOutPort;
	sc_fifo_out<SIMAGE> MagToAppOutPort;
	SIMAGE mag, dx, dy;
	IMAGE nms;

	void non_max_supp(short *mag, short *gradx, short *grady, int nrows, int ncols,
		unsigned char *result);
	void start_non_max_supp();

	SC_CTOR(NON_MAX_SUPP) {
		SC_THREAD(start_non_max_supp);
		SET_STACK_SIZE;
	}
};


SC_MODULE(APPLY_HYSTERESIS) {
	sc_fifo_in<SIMAGE> MagInPort;
	sc_fifo_in<IMAGE> NmsInPort;
	sc_fifo_out<IMAGE> EdgeOutPort;
	SIMAGE mag;
	IMAGE nms, edge;

	void apply_hysteresis(short int *mag, unsigned char *nms, int rows, int cols,
		float tlow, float thigh, unsigned char *edge);
	void follow_edges(unsigned char *edgemapptr, short *edgemagptr, short lowval,
		int cols);
	void start_apply_hysteresis();

	SC_CTOR(APPLY_HYSTERESIS) {
		SC_THREAD(start_apply_hysteresis);
		SET_STACK_SIZE;
	}
};


SC_MODULE(DUT) {
	sc_fifo_in<IMAGE> ImgInPort;
	sc_fifo_out<IMAGE> ImgOutPort;

	sc_fifo<SIMAGE> sdimChannel;
	sc_fifo<SIMAGE> dxmagChannel;
	sc_fifo<SIMAGE> dymagChannel;
	sc_fifo<SIMAGE> dxnonChannel;
	sc_fifo<SIMAGE> dynonChannel;
	sc_fifo<SIMAGE> magnonChannel;
	sc_fifo<SIMAGE> magappChannel;
	sc_fifo<IMAGE> nmsChannel;

	GAUSSIAN_SMOOTH gaussian_smoothModule;
	DERIVATIVE_X_Y derrivative_x_yModule;
	MAGNITUDE_X_Y magnitude_x_yModule;
	NON_MAX_SUPP non_max_suppModule;
	APPLY_HYSTERESIS apply_hysteresisModule;

	void before_end_of_elaboration() {
		gaussian_smoothModule.ImgInPort.bind(ImgInPort);
		gaussian_smoothModule.SdimOutPort.bind(sdimChannel);

		derrivative_x_yModule.SdimInPort.bind(sdimChannel);
		derrivative_x_yModule.DxToMagOutPort.bind(dxmagChannel);
		derrivative_x_yModule.DyToMagOutPort.bind(dymagChannel);

		magnitude_x_yModule.DxInPort.bind(dxmagChannel);
		magnitude_x_yModule.DyInPort.bind(dymagChannel);
		magnitude_x_yModule.MagToNonOutPort.bind(magnonChannel);
		magnitude_x_yModule.DxToNonOutPort.bind(dxnonChannel);
		magnitude_x_yModule.DyToNonOutPort.bind(dynonChannel);

		non_max_suppModule.MagInPort.bind(magnonChannel);
		non_max_suppModule.DxInPort.bind(dxnonChannel);
		non_max_suppModule.DyInPort.bind(dynonChannel);
		non_max_suppModule.NmsOutPort.bind(nmsChannel);
		non_max_suppModule.MagToAppOutPort.bind(magappChannel);

		apply_hysteresisModule.MagInPort.bind(magappChannel);
		apply_hysteresisModule.NmsInPort.bind(nmsChannel);
		apply_hysteresisModule.EdgeOutPort.bind(ImgOutPort);
	}

	SC_CTOR(DUT) :
		sdimChannel("sdimChannel", 1), dxmagChannel("dxmagChannel", 1), dymagChannel("dymagChannel", 1),
		dxnonChannel("dxnonChannel", 1), dynonChannel("dynonChannel", 1), magnonChannel("magnonChannel", 1),
		magappChannel("magappChannel", 1), nmsChannel("nmsChannel", 1),
		gaussian_smoothModule("gaussian_smoothModule"), derrivative_x_yModule("derrivative_x_yModule"),
		magnitude_x_yModule("magnitude_x_yModule"), non_max_suppModule("non_max_suppModule"),
		apply_hysteresisModule("apply_hysteresisModule") {

	}
};


//Platform
SC_MODULE(Plat)
{
	sc_fifo_in<IMAGE> ImgInPort;
	sc_fifo_out<IMAGE> ImgOutPort;
	sc_fifo<IMAGE> q1;
	sc_fifo<IMAGE> q2;

	DataIn din;
	DUT canny;
	DataOut dout;

	void before_end_of_elaboration() {
		din.ImgInPort.bind(ImgInPort);
		din.ImgOutPort.bind(q1);
		canny.ImgInPort.bind(q1);
		canny.ImgOutPort.bind(q2);
		dout.ImgInPort.bind(q2);
		dout.ImgOutPort.bind(ImgOutPort);
	}

	SC_CTOR(Plat) :
		// channel 2nd para is size of buffer
		q1("q1", 1), q2("q2", 1), din("din"), canny("canny"), dout("dout") {
	}
};

SC_MODULE(Top)
{
	sc_fifo<IMAGE> q1;
	sc_fifo<IMAGE> q2;
	sc_fifo<sc_time> ft;

	Stim stim1;
	Plat plat1;
	Mon mon1;

	void before_end_of_elaboration() {
		stim1.ImgOutPort.bind(q1);
		stim1.FTOutPort.bind(ft);
		plat1.ImgInPort.bind(q1);
		plat1.ImgOutPort.bind(q2);
		mon1.ImgInPort.bind(q2);
		mon1.FTInPort.bind(ft);
	}

	SC_CTOR(Top) :
		q1("q1", 2), q2("q2", 2), ft("ft", 2), stim1("stim1"), plat1("plat1"), mon1("mon1") {
	}
};


Top top("top");
int sc_main(int argc, char* argv[]) {
	sc_start();
	return 0;
}


/**************************************************************************************************/
/*                                          Stim                                                  */
/**************************************************************************************************/
/******************************************************************************
* Function: read_pgm_image
* Purpose: This function reads in an image in PGM format. The image can be
* read in from either a file or from standard input. The image is only read
* from standard input when infilename = NULL. Because the PGM format includes
* the number of columns and the number of rows in the image, these are read
* from the file. Memory to store the image is allocated OUTSIDE this function.
* The found image size is checked against the expected rows and cols.
* All comments in the header are discarded in the process of reading the
* image. Upon failure, this function returns 0, upon sucess it returns 1.
******************************************************************************/
int Stim::read_pgm_image(const char *infilename, unsigned char *image, int rows, int cols)
{
	FILE *fp;
	char buf[71];
	int r, c;

	/***************************************************************************
	* Open the input image file for reading if a filename was given. If no
	* filename was provided, set fp to read from standard input.
	***************************************************************************/
	if (infilename == NULL) fp = stdin;
	else {
		if ((fp = fopen(infilename, "r")) == NULL) {
			fprintf(stderr, "Error reading the file %s in read_pgm_image().\n",
				infilename);
			return(0);
		}
	}

	/***************************************************************************
	* Verify that the image is in PGM format, read in the number of columns
	* and rows in the image and scan past all of the header information.
	***************************************************************************/
	fgets(buf, 70, fp);
	if (strncmp(buf, "P5", 2) != 0) {
		fprintf(stderr, "The file %s is not in PGM format in ", infilename);
		fprintf(stderr, "read_pgm_image().\n");
		if (fp != stdin) fclose(fp);
		return(0);
	}
	do { fgets(buf, 70, fp); } while (buf[0] == '#');  /* skip all comment lines */
	sscanf(buf, "%d %d", &c, &r);
	if (c != cols || r != rows) {
		fprintf(stderr, "The file %s is not a %d by %d image in ", infilename, cols, rows);
		fprintf(stderr, "read_pgm_image().\n");
		if (fp != stdin) fclose(fp);
		return(0);
	}
	do { fgets(buf, 70, fp); } while (buf[0] == '#');  /* skip all comment lines */

	/***************************************************************************
	* Read the image from the file.
	***************************************************************************/
	if ((unsigned)rows != fread(image, cols, rows, fp)) {
		fprintf(stderr, "Error reading the image data in read_pgm_image().\n");
		if (fp != stdin) fclose(fp);
		return(0);
	}

	if (fp != stdin) fclose(fp);
	return(1);
}
/******************************************************************************
* Function: read_img_in
* Purpose: This function is 'main' function of module Stim, it call read_pgm_image IMG_NUM
* times
******************************************************************************/
void Stim::read_img_in()
{
	char infilename[70];
	int i = 0, n = 0;
	for (i = 0; i < IMG_NUM; i++) {
		n = i % AVAIL_IMG;
		sprintf(infilename, IMG_IN, n + 1);
		/****************************************************************************
		* Read in the image. This read function allocates memory for the image.
		****************************************************************************/
		if (VERBOSE) printf("Reading the image %s.\n", infilename);
		if (read_pgm_image(infilename, img, ROWS, COLS) == 0) {
			fprintf(stderr, "Error reading the input image, %s.\n", infilename);
			exit(1);
		}
		ImgOutPort.write(img);
		startTime = sc_time_stamp();
		printf("%7d ms: Stimulation sent frame %02d.\n", (int)startTime.to_seconds()*1000, i+1);
		FTOutPort.write(startTime);
	}
}


/**************************************************************************************************/
/*                                          Mon                                                   */
/**************************************************************************************************/
/******************************************************************************
* Function: write_pgm_image
* Purpose: This function writes an image in PGM format. The file is either
* written to the file specified by outfilename or to standard output if
* outfilename = NULL. A comment can be written to the header if coment != NULL.
******************************************************************************/
int Mon::write_pgm_image(const char *outfilename, unsigned char *image, int rows,
	int cols, const char *comment, int maxval)
{
	FILE *fp;

	/***************************************************************************
	* Open the output image file for writing if a filename was given. If no
	* filename was provided, set fp to write to standard output.
	***************************************************************************/
	if (outfilename == NULL) fp = stdout;
	else {
		if ((fp = fopen(outfilename, "w")) == NULL) {
			fprintf(stderr, "Error writing the file %s in write_pgm_image().\n",
				outfilename);
			return(0);
		}
	}

	/***************************************************************************
	* Write the header information to the PGM file.
	***************************************************************************/
	fprintf(fp, "P5\n%d %d\n", cols, rows);
	if (comment != NULL)
		if (strlen(comment) <= 70) fprintf(fp, "# %s\n", comment);
	fprintf(fp, "%d\n", maxval);

	/***************************************************************************
	* Write the image data to the file.
	***************************************************************************/
	if ((unsigned)rows != fwrite(image, cols, rows, fp)) {
		fprintf(stderr, "Error writing the image data in write_pgm_image().\n");
		if (fp != stdout) fclose(fp);
		return(0);
	}

	if (fp != stdout) fclose(fp);
	return(1);
}
/******************************************************************************
* Function: read_img_in
* Purpose: This function is 'main' function of module Mon, it call write_pgm_image
******************************************************************************/
void Mon::print_img_out()
{
	char outfilename[128];
	int i = 0, n = 0;
	sc_time prev = SC_ZERO_TIME;
	sc_time curr = SC_ZERO_TIME;

	for (i = 0; i < IMG_NUM; i++) {
		FTInPort.read(startTime);
		ImgInPort.read(img);

		n = i % AVAIL_IMG;
		sprintf(outfilename, IMG_OUT, n + 1);
		if (VERBOSE) printf("Writing the edge iname in the file %s.\n", outfilename);
		if (write_pgm_image(outfilename, img, ROWS, COLS, "", 255) == 0) {
			fprintf(stderr, "Error writing the edge image, %s.\n", outfilename);
			exit(1);
		}
		printf("%7d ms: Monitor recerived frame %02d with %7d ms delay.\n", (int)(sc_time_stamp().to_seconds()*1000), i+1, (int)((sc_time_stamp()-startTime).to_seconds()*1000));
		prev = curr;
		curr = sc_time_stamp();
		if(i>=1){
			printf("%7d ms: %.3f seconds after previous frame %02d, %.3f FPS.\n", (int)(sc_time_stamp().to_seconds()*1000), (float)(curr-prev).to_seconds(), i, 1/(curr-prev).to_seconds());
		}	
	}
	printf("%7d ms: Monitor exits stimulation.\n", (int)(sc_time_stamp().to_seconds()*1000));
	sc_stop();
}


/**************************************************************************************************/
/*                                          DUT                                                   */
/**************************************************************************************************/
/******************************************************************************************/
/*                                       GAUSSIAN_SMOOTH                                  */
/******************************************************************************************/
/****************************************************************************
* Perform gaussian smoothing on the image using the input standard
* deviation.
****************************************************************************/
/****************************************************************************
* Create a 1-dimensional gaussian smoothing kernel.
****************************************************************************/

void GAUSSIAN_KERNEL::make_gaussian_kernel(float sigma, float *kernel, int *windowsize)
{
	int i, center;
	float x, fx, sum = 0.0;

	*windowsize = 1 + 2 * ceil(2.5 * sigma);
	center = (*windowsize) / 2;

	if (VERBOSE) printf("      The kernel has %d elements.\n", *windowsize);

	for (i = 0; i < (*windowsize); i++) {
		x = (float)(i - center);
		fx = pow(2.71828, -0.5*x*x / (sigma*sigma)) / (sigma * sqrt(6.2831853));
		kernel[i] = fx;
		sum += fx;
	}

	for (i = 0; i < (*windowsize); i++) kernel[i] /= sum;

	if (VERBOSE) {
		printf("The filter coefficients are:\n");
		for (i = 0; i < (*windowsize); i++)
			printf("kernel[%d] = %f\n", i, kernel[i]);
	}
}

void GAUSSIAN_KERNEL::start_make_gaussian_kernel()
{
	while (1) {
		wait(0, SC_MS);
		make_gaussian_kernel(SIGMA, kernel, &wsize);
		WsOutToBlurXPort.write(wsize);
		KernelToBlurXOutPort.write(kernel);
	}
}

void BLURX::blurx1()
{
	while(1){
		wait(ef);
		int r, c, cc;     /* Counter variables. */
		int center = wsize / 2;            /* Half of the windowsize. */
		float dot;            /* Dot product summing variable. */
		float sum;            /* Sum of the kernel weights variable. */
		// put paras directly into funtion
		int rows = ROWS;
		int cols = COLS;

		/****************************************************************************
		* Blur in the x - direction.
		****************************************************************************/
		if (VERBOSE) printf("   Bluring the image in the X-direction.\n");
		for (r = 0; r <= (rows/4)*1-1; r++) {
			for (c = 0; c < cols; c++) {
				dot = 0.0;
				sum = 0.0;
				for (cc = (-center); cc <= center; cc++) {
					if (((c + cc) >= 0) && ((c + cc) < cols)) {
						dot += (float)img[r*cols + (c + cc)] * kernel[center + cc];
						sum += kernel[center + cc];
					}
				}
				tempim[r*cols + c] = dot / sum;
			}
		}
		wait(1710/4, SC_MS);
		e1.notify(SC_ZERO_TIME);
	}
}

void BLURX::blurx2()
{
	while(1){
		wait(ef);
		int r, c, cc;     /* Counter variables. */
		int center = wsize / 2;            /* Half of the windowsize. */
		float dot;            /* Dot product summing variable. */
		float sum;            /* Sum of the kernel weights variable. */
		int rows = ROWS;
		int cols = COLS;

		/****************************************************************************
		* Blur in the x - direction.
		****************************************************************************/
		if (VERBOSE) printf("   Bluring the image in the X-direction.\n");
		for (r = (rows/4)*1; r <= (rows/4)*2-1; r++) {
			for (c = 0; c < cols; c++) {
				dot = 0.0;
				sum = 0.0;
				for (cc = (-center); cc <= center; cc++) {
					if (((c + cc) >= 0) && ((c + cc) < cols)) {
						dot += (float)img[r*cols + (c + cc)] * kernel[center + cc];
						sum += kernel[center + cc];
					}
				}
				tempim[r*cols + c] = dot / sum;
			}
		}
		wait(1710/4, SC_MS);
		e2.notify(SC_ZERO_TIME);
	}
}

void BLURX::blurx3()
{
	while(1){
		wait(ef);
		int r, c, cc;     /* Counter variables. */
		int center = wsize / 2;            /* Half of the windowsize. */
		float dot;            /* Dot product summing variable. */
		float sum;            /* Sum of the kernel weights variable. */
		int rows = ROWS;
		int cols = COLS;

		/****************************************************************************
		* Blur in the x - direction.
		****************************************************************************/
		if (VERBOSE) printf("   Bluring the image in the X-direction.\n");
		for (r = (rows/4)*2; r <= (rows/4)*3-1; r++) {
			for (c = 0; c < cols; c++) {
				dot = 0.0;
				sum = 0.0;
				for (cc = (-center); cc <= center; cc++) {
					if (((c + cc) >= 0) && ((c + cc) < cols)) {
						dot += (float)img[r*cols + (c + cc)] * kernel[center + cc];
						sum += kernel[center + cc];
					}
				}
				tempim[r*cols + c] = dot / sum;
			}
		}
		wait(1710/4, SC_MS);
		e3.notify(SC_ZERO_TIME);
	}
}

void BLURX::blurx4()
{
	while(1){
		wait(ef);
		int r, c, cc;     /* Counter variables. */
		int center = wsize / 2;            /* Half of the windowsize. */
		float dot;            /* Dot product summing variable. */
		float sum;            /* Sum of the kernel weights variable. */
		int rows = ROWS;
		int cols = COLS;

		/****************************************************************************
		* Blur in the x - direction.
		****************************************************************************/
		if (VERBOSE) printf("   Bluring the image in the X-direction.\n");
		for (r = (rows/4)*3; r <= (rows/4)*4-1; r++) {
			for (c = 0; c < cols; c++) {
				dot = 0.0;
				sum = 0.0;
				for (cc = (-center); cc <= center; cc++) {
					if (((c + cc) >= 0) && ((c + cc) < cols)) {
						dot += (float)img[r*cols + (c + cc)] * kernel[center + cc];
						sum += kernel[center + cc];
					}
				}
				tempim[r*cols + c] = dot / sum;
			}
		}
		wait(1710/4, SC_MS);
		e4.notify(SC_ZERO_TIME);
	}
}

void BLURX::start_blurx()
{
	while (1) {
		ImgInPort.read(img);
		KernelInPort.read(kernel);
		WsInPort.read(wsize);

		// to make them start together
		ef.notify(SC_ZERO_TIME);
		wait(e1 & e2 & e3 & e4);

		TempimOutPort.write(tempim);
		WsOutToBlurYPort.write(wsize);
		KernelToBlurYOutPort.write(kernel);
	}
}

void BLURY::blury1()
{
	while(1){
		wait(ef);
		int r, c, rr;     /* Counter variables. */
		int center = wsize / 2;            /* Half of the windowsize. */
		float dot;            /* Dot product summing variable. */
		float sum;            /* Sum of the kernel weights variable. */
		int rows = ROWS;
		int cols = COLS;

		/****************************************************************************
		* Blur in the y - direction.
		****************************************************************************/
		if (VERBOSE) printf("   Bluring the image in the Y-direction.\n");
		for (c = 0; c <= (cols/4)*1-1; c++) {
			for (r = 0; r < rows; r++) {
				sum = 0.0;
				dot = 0.0;
				for (rr = (-center); rr <= center; rr++) {
					if (((r + rr) >= 0) && ((r + rr) < rows)) {
						dot += tempim[(r + rr)*cols + c] * kernel[center + rr];
						sum += kernel[center + rr];
					}
				}
				smoothedim[r*cols + c] = (short int)(dot*BOOSTBLURFACTOR / sum + 0.5);
			}
		}
		wait(1820/4, SC_MS);
		e1.notify(SC_ZERO_TIME);
	}
}

void BLURY::blury2()
{
	while(1){
		wait(ef);
		int r, c, rr;     /* Counter variables. */
		int center = wsize / 2;            /* Half of the windowsize. */
		float dot;            /* Dot product summing variable. */
		float sum;            /* Sum of the kernel weights variable. */
		int rows = ROWS;
		int cols = COLS;

		/****************************************************************************
		* Blur in the y - direction.
		****************************************************************************/
		if (VERBOSE) printf("   Bluring the image in the Y-direction.\n");
		for (c = (cols/4)*1; c <= (cols/4)*2-1; c++) {
			for (r = 0; r < rows; r++) {
				sum = 0.0;
				dot = 0.0;
				for (rr = (-center); rr <= center; rr++) {
					if (((r + rr) >= 0) && ((r + rr) < rows)) {
						dot += tempim[(r + rr)*cols + c] * kernel[center + rr];
						sum += kernel[center + rr];
					}
				}
				smoothedim[r*cols + c] = (short int)(dot*BOOSTBLURFACTOR / sum + 0.5);
			}
		}
		wait(1820/4, SC_MS);
		e2.notify(SC_ZERO_TIME);
	}
}

void BLURY::blury3()
{
	while(1){
		wait(ef);
		int r, c, rr;     /* Counter variables. */
		int center = wsize / 2;            /* Half of the windowsize. */
		float dot;            /* Dot product summing variable. */
		float sum;            /* Sum of the kernel weights variable. */
		int rows = ROWS;
		int cols = COLS;

		/****************************************************************************
		* Blur in the y - direction.
		****************************************************************************/
		if (VERBOSE) printf("   Bluring the image in the Y-direction.\n");
		for (c = (cols/4)*2; c <= (cols/4)*3-1; c++) {
			for (r = 0; r < rows; r++) {
				sum = 0.0;
				dot = 0.0;
				for (rr = (-center); rr <= center; rr++) {
					if (((r + rr) >= 0) && ((r + rr) < rows)) {
						dot += tempim[(r + rr)*cols + c] * kernel[center + rr];
						sum += kernel[center + rr];
					}
				}
				smoothedim[r*cols + c] = (short int)(dot*BOOSTBLURFACTOR / sum + 0.5);
			}
		}
		wait(1820/4, SC_MS);
		e3.notify(SC_ZERO_TIME);
	}
}

void BLURY::blury4()
{
	while(1){
		wait(ef);
		int r, c, rr;     /* Counter variables. */
		int center = wsize / 2;            /* Half of the windowsize. */
		float dot;            /* Dot product summing variable. */
		float sum;            /* Sum of the kernel weights variable. */
		int rows = ROWS;
		int cols = COLS;

		/****************************************************************************
		* Blur in the y - direction.
		****************************************************************************/
		if (VERBOSE) printf("   Bluring the image in the Y-direction.\n");
		for (c = (cols/4)*3; c <= (cols/4)*4-1; c++) {
			for (r = 0; r < rows; r++) {
				sum = 0.0;
				dot = 0.0;
				for (rr = (-center); rr <= center; rr++) {
					if (((r + rr) >= 0) && ((r + rr) < rows)) {
						dot += tempim[(r + rr)*cols + c] * kernel[center + rr];
						sum += kernel[center + rr];
					}
				}
				smoothedim[r*cols + c] = (short int)(dot*BOOSTBLURFACTOR / sum + 0.5);
			}
		}
		wait(1820/4, SC_MS);
		e4.notify(SC_ZERO_TIME);
	}
}

void BLURY::start_blury()
{
	while (1) {
		TempimInPort.read(tempim);
		KernelInPort.read(kernel);
		WsInPort.read(wsize);

		ef.notify(SC_ZERO_TIME);
		wait(e1 & e2 & e3 & e4);

		SdimOutPort.write(smoothedim);
	}

}

/******************************************************************************************/
/*                                   Other Module in DUT                                  */
/******************************************************************************************/
/****************************************************************************
* Compute the first derivative in the x and y directions.
****************************************************************************/
void DERIVATIVE_X_Y::derrivative_x_y(short int *smoothedim, int rows, int cols,
	short int *delta_x, short int *delta_y)
{
	int r, c, pos;

	/****************************************************************************
	* Compute the x-derivative. Adjust the derivative at the borders to avoid
	* losing pixels.
	****************************************************************************/
	if (VERBOSE) printf("   Computing the X-direction derivative.\n");
	for (r = 0; r < rows; r++) {
		pos = r * cols;
		delta_x[pos] = smoothedim[pos + 1] - smoothedim[pos];
		pos++;
		for (c = 1; c < (cols - 1); c++, pos++) {
			delta_x[pos] = smoothedim[pos + 1] - smoothedim[pos - 1];
		}
		delta_x[pos] = smoothedim[pos] - smoothedim[pos - 1];
	}

	/****************************************************************************
	* Compute the y-derivative. Adjust the derivative at the borders to avoid
	* losing pixels.
	****************************************************************************/
	if (VERBOSE) printf("   Computing the Y-direction derivative.\n");
	for (c = 0; c < cols; c++) {
		pos = c;
		delta_y[pos] = smoothedim[pos + cols] - smoothedim[pos];
		pos += cols;
		for (r = 1; r < (rows - 1); r++, pos += cols) {
			delta_y[pos] = smoothedim[pos + cols] - smoothedim[pos - cols];
		}
		delta_y[pos] = smoothedim[pos] - smoothedim[pos - cols];
	}
}

void DERIVATIVE_X_Y::start_derrivative_x_y()
{
	while (1) {
		SdimInPort.read(sdim);
		wait(480, SC_MS);

		derrivative_x_y(sdim, ROWS, COLS, dx, dy);
		DxToMagOutPort.write(dx);
		DyToMagOutPort.write(dy);
	}
}

/****************************************************************************
* Compute the magnitude of the gradient.
****************************************************************************/
void MAGNITUDE_X_Y::magnitude_x_y(short int *delta_x, short int *delta_y, int rows, int cols,
	short int *magnitude)
{
	int r, c, pos, sq1, sq2;

	for (r = 0, pos = 0; r < rows; r++) {
		for (c = 0; c < cols; c++, pos++) {
			sq1 = (int)delta_x[pos] * (int)delta_x[pos];
			sq2 = (int)delta_y[pos] * (int)delta_y[pos];
			magnitude[pos] = (short)(0.5 + sqrt((float)sq1 + (float)sq2));
		}
	}

}

void MAGNITUDE_X_Y::start_magnitude_x_y()
{
	while (1) {
		DxInPort.read(dx);
		DyInPort.read(dy);
		wait(1030, SC_MS);

		magnitude_x_y(dx, dy, ROWS, COLS, mag);
		MagToNonOutPort.write(mag);
		DxToNonOutPort.write(dx);
		DyToNonOutPort.write(dy);
	}
}

/****************************************************************************
* Perform non-maximal suppression.
****************************************************************************/
void NON_MAX_SUPP::non_max_supp(short *mag, short *gradx, short *grady, int nrows, int ncols,
	unsigned char *result)
{
	int rowcount, colcount, count;
	short *magrowptr, *magptr;
	short *gxrowptr, *gxptr;
	short *gyrowptr, *gyptr, z1, z2;
	short m00, gx, gy;
	float mag1, mag2, xperp, yperp;
	unsigned char *resultrowptr, *resultptr;

	/****************************************************************************
	* Zero the edges of the result image.
	****************************************************************************/
	for (count = 0, resultrowptr = result, resultptr = result + ncols * (nrows - 1);
		count < ncols; resultptr++, resultrowptr++, count++) {
		*resultrowptr = *resultptr = (unsigned char)0;
	}

	for (count = 0, resultptr = result, resultrowptr = result + ncols - 1;
		count < nrows; count++, resultptr += ncols, resultrowptr += ncols) {
		*resultptr = *resultrowptr = (unsigned char)0;
	}

	/****************************************************************************
	* Suppress non-maximum points.
	****************************************************************************/
	for (rowcount = 1, magrowptr = mag + ncols + 1, gxrowptr = gradx + ncols + 1,
		gyrowptr = grady + ncols + 1, resultrowptr = result + ncols + 1;
		rowcount <= nrows - 2;   // bug fix 3/29/17, RD
		rowcount++, magrowptr += ncols, gyrowptr += ncols, gxrowptr += ncols,
		resultrowptr += ncols) {
		for (colcount = 1, magptr = magrowptr, gxptr = gxrowptr, gyptr = gyrowptr,
			resultptr = resultrowptr; colcount <= ncols - 2; // bug fix 3/29/17, RD
			colcount++, magptr++, gxptr++, gyptr++, resultptr++) {
			m00 = *magptr;
			if (m00 == 0) {
				*resultptr = (unsigned char)NOEDGE;
			}
			else {
				xperp = -(gx = *gxptr) / ((float)m00);
				yperp = (gy = *gyptr) / ((float)m00);
			}

			if (gx >= 0) {
				if (gy >= 0) {
					if (gx >= gy)
					{
						/* 111 */
						/* Left point */
						z1 = *(magptr - 1);
						z2 = *(magptr - ncols - 1);

						mag1 = (m00 - z1)*xperp + (z2 - z1)*yperp;

						/* Right point */
						z1 = *(magptr + 1);
						z2 = *(magptr + ncols + 1);

						mag2 = (m00 - z1)*xperp + (z2 - z1)*yperp;
					}
					else
					{
						/* 110 */
						/* Left point */
						z1 = *(magptr - ncols);
						z2 = *(magptr - ncols - 1);

						mag1 = (z1 - z2)*xperp + (z1 - m00)*yperp;

						/* Right point */
						z1 = *(magptr + ncols);
						z2 = *(magptr + ncols + 1);

						mag2 = (z1 - z2)*xperp + (z1 - m00)*yperp;
					}
				}
				else
				{
					if (gx >= -gy)
					{
						/* 101 */
						/* Left point */
						z1 = *(magptr - 1);
						z2 = *(magptr + ncols - 1);

						mag1 = (m00 - z1)*xperp + (z1 - z2)*yperp;

						/* Right point */
						z1 = *(magptr + 1);
						z2 = *(magptr - ncols + 1);

						mag2 = (m00 - z1)*xperp + (z1 - z2)*yperp;
					}
					else
					{
						/* 100 */
						/* Left point */
						z1 = *(magptr + ncols);
						z2 = *(magptr + ncols - 1);

						mag1 = (z1 - z2)*xperp + (m00 - z1)*yperp;

						/* Right point */
						z1 = *(magptr - ncols);
						z2 = *(magptr - ncols + 1);

						mag2 = (z1 - z2)*xperp + (m00 - z1)*yperp;
					}
				}
			}
			else
			{
				if ((gy = *gyptr) >= 0)
				{
					if (-gx >= gy)
					{
						/* 011 */
						/* Left point */
						z1 = *(magptr + 1);
						z2 = *(magptr - ncols + 1);

						mag1 = (z1 - m00)*xperp + (z2 - z1)*yperp;

						/* Right point */
						z1 = *(magptr - 1);
						z2 = *(magptr + ncols - 1);

						mag2 = (z1 - m00)*xperp + (z2 - z1)*yperp;
					}
					else
					{
						/* 010 */
						/* Left point */
						z1 = *(magptr - ncols);
						z2 = *(magptr - ncols + 1);

						mag1 = (z2 - z1)*xperp + (z1 - m00)*yperp;

						/* Right point */
						z1 = *(magptr + ncols);
						z2 = *(magptr + ncols - 1);

						mag2 = (z2 - z1)*xperp + (z1 - m00)*yperp;
					}
				}
				else
				{
					if (-gx > -gy)
					{
						/* 001 */
						/* Left point */
						z1 = *(magptr + 1);
						z2 = *(magptr + ncols + 1);

						mag1 = (z1 - m00)*xperp + (z1 - z2)*yperp;

						/* Right point */
						z1 = *(magptr - 1);
						z2 = *(magptr - ncols - 1);

						mag2 = (z1 - m00)*xperp + (z1 - z2)*yperp;
					}
					else
					{
						/* 000 */
						/* Left point */
						z1 = *(magptr + ncols);
						z2 = *(magptr + ncols + 1);

						mag1 = (z2 - z1)*xperp + (m00 - z1)*yperp;

						/* Right point */
						z1 = *(magptr - ncols);
						z2 = *(magptr - ncols - 1);

						mag2 = (z2 - z1)*xperp + (m00 - z1)*yperp;
					}
				}
			}

			/* Now determine if the current point is a maximum point */

			if ((mag1 > 0.0) || (mag2 > 0.0))
			{
				*resultptr = (unsigned char)NOEDGE;
			}
			else
			{
				if (mag2 == 0.0)
					*resultptr = (unsigned char)NOEDGE;
				else
					*resultptr = (unsigned char)POSSIBLE_EDGE;
			}
		}
	}
}

void NON_MAX_SUPP::start_non_max_supp()
{
	while (1) {
		MagInPort.read(mag);
		DxInPort.read(dx);
		DyInPort.read(dy);
		wait(830, SC_MS);

		non_max_supp(mag, dx, dy, ROWS, COLS, nms);
		NmsOutPort.write(nms);
		MagToAppOutPort.write(mag);

	}
}

void APPLY_HYSTERESIS::follow_edges(unsigned char *edgemapptr, short *edgemagptr, short lowval,
	int cols)
{
	short *tempmagptr;
	unsigned char *tempmapptr;
	int i;
	int x[8] = { 1,1,0,-1,-1,-1,0,1 },
		y[8] = { 0,1,1,1,0,-1,-1,-1 };

	for (i = 0; i < 8; i++) {
		tempmapptr = edgemapptr - y[i] * cols + x[i];
		tempmagptr = edgemagptr - y[i] * cols + x[i];

		if ((*tempmapptr == POSSIBLE_EDGE) && (*tempmagptr > lowval)) {
			*tempmapptr = (unsigned char)EDGE;
			follow_edges(tempmapptr, tempmagptr, lowval, cols);
		}
	}
}

/****************************************************************************
* Use hysteresis to mark the edge pixels.
****************************************************************************/
void APPLY_HYSTERESIS::apply_hysteresis(short int *mag, unsigned char *nms, int rows, int cols,
	float tlow, float thigh, unsigned char *edge)
{
	int r, c, pos, numedges, highcount, lowthreshold, highthreshold, hist[32768];
	short int maximum_mag;

	/****************************************************************************
	* Initialize the edge map to possible edges everywhere the non-maximal
	* suppression suggested there could be an edge except for the border. At
	* the border we say there can not be an edge because it makes the
	* follow_edges algorithm more efficient to not worry about tracking an
	* edge off the side of the image.
	****************************************************************************/
	for (r = 0, pos = 0; r < rows; r++) {
		for (c = 0; c < cols; c++, pos++) {
			if (nms[pos] == POSSIBLE_EDGE) edge[pos] = POSSIBLE_EDGE;
			else edge[pos] = NOEDGE;
		}
	}

	for (r = 0, pos = 0; r < rows; r++, pos += cols) {
		edge[pos] = NOEDGE;
		edge[pos + cols - 1] = NOEDGE;
	}
	pos = (rows - 1) * cols;
	for (c = 0; c < cols; c++, pos++) {
		edge[c] = NOEDGE;
		edge[pos] = NOEDGE;
	}

	/****************************************************************************
	* Compute the histogram of the magnitude image. Then use the histogram to
	* compute hysteresis thresholds.
	****************************************************************************/
	for (r = 0; r < 32768; r++) hist[r] = 0;
	for (r = 0, pos = 0; r < rows; r++) {
		for (c = 0; c < cols; c++, pos++) {
			if (edge[pos] == POSSIBLE_EDGE) hist[mag[pos]]++;
		}
	}

	/****************************************************************************
	* Compute the number of pixels that passed the nonmaximal suppression.
	****************************************************************************/
	for (r = 1, numedges = 0; r < 32768; r++) {
		if (hist[r] != 0) maximum_mag = r;
		numedges += hist[r];
	}

	highcount = (int)(numedges * thigh + 0.5);

	/****************************************************************************
	* Compute the high threshold value as the (100 * thigh) percentage point
	* in the magnitude of the gradient histogram of all the pixels that passes
	* non-maximal suppression. Then calculate the low threshold as a fraction
	* of the computed high threshold value. John Canny said in his paper
	* "A Computational Approach to Edge Detection" that "The ratio of the
	* high to low threshold in the implementation is in the range two or three
	* to one." That means that in terms of this implementation, we should
	* choose tlow ~= 0.5 or 0.33333.
	****************************************************************************/
	r = 1;
	numedges = hist[1];
	while ((r < (maximum_mag - 1)) && (numedges < highcount)) {
		r++;
		numedges += hist[r];
	}
	highthreshold = r;
	lowthreshold = (int)(highthreshold * tlow + 0.5);

	if (VERBOSE) {
		printf("The input low and high fractions of %f and %f computed to\n",
			tlow, thigh);
		printf("magnitude of the gradient threshold values of: %d %d\n",
			lowthreshold, highthreshold);
	}

	/****************************************************************************
	* This loop looks for pixels above the highthreshold to locate edges and
	* then calls follow_edges to continue the edge.
	****************************************************************************/
	for (r = 0, pos = 0; r < rows; r++) {
		for (c = 0; c < cols; c++, pos++) {
			if ((edge[pos] == POSSIBLE_EDGE) && (mag[pos] >= highthreshold)) {
				edge[pos] = EDGE;
				follow_edges((edge + pos), (mag + pos), lowthreshold, cols);
			}
		}
	}

	/****************************************************************************
	* Set all the remaining possible edges to non-edges.
	****************************************************************************/
	for (r = 0, pos = 0; r < rows; r++) {
		for (c = 0; c < cols; c++, pos++) if (edge[pos] != EDGE) edge[pos] = NOEDGE;
	}
}

void APPLY_HYSTERESIS::start_apply_hysteresis()
{
	while (1) {
		MagInPort.read(mag);
		NmsInPort.read(nms);
		wait(670, SC_MS);

		apply_hysteresis(mag, nms, ROWS, COLS, TLOW, THIGH, edge);
		EdgeOutPort.write(edge);
	}
}