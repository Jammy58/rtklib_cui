// rtklib_console.cpp : 定义控制台应用程序的入口点。

/************ 注意 ***************
未修改rtklib代码时，生成这个工程即可 
*********************************/

#include "stdafx.h"
#include "rtklib.h"
#define INFILEMAX 5
#define BUFSIZE 1024

#define IN_ATX08		"E:/0.data/IGS/igs08.atx"
#define IN_FILE_1		"E:/0.data/PPP/2016428/gmsd1190.16o"
#define IN_FILE_2		"E:/0.data/PPP/2016428/brdm1190.16p"
#define IN_FILE_3		"E:/0.data/PPP/2016428/com18944.sp3"
#define IN_FILE_4		"E:/0.data/PPP/2016428/com18944.clk"
#define OUT_FILE_POS	"E:/0.data/PPP/2016428/_gmsd1190.pos"

void initFopt(filopt_t & opt) {
	// to do： auto select the ANTEX file according to the obs date

	strcpy(opt.satantp, IN_ATX08);
}
void initPopt(prcopt_t & opt) {
	opt.mode = PMODE_PPP_STATIC; /*PMODE_PPP_KINEMA*/
	opt.nf = 3;
	opt.navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_CMP;
	opt.elmin = 10.0*D2R;
	opt.sateph = EPHOPT_PREC;
	opt.modear = 0; /*float mode*/
	opt.glomodear = 0;
	opt.bdsmodear = 0;
	opt.ionoopt = IONOOPT_IFLC;
	opt.tropopt = TROPOPT_EST; /*TROPOPT_ESTG*/
	/* 未改正海潮 */
}			
void initSopt(solopt_t & opt) {
	
}
void initIn_output(char*infile[], int& n, char *outfile) {
	for (int i = 0; i < INFILEMAX; i++)
	{
		infile[i] = (char*)malloc(sizeof(char)*BUFSIZE);
		*infile[i] = '\0';
	}
	strcpy(infile[0], IN_FILE_1);
	strcpy(infile[1], IN_FILE_2);
	strcpy(infile[2], IN_FILE_3);
	strcpy(infile[3], IN_FILE_4);
	
	n = 0;
	while (strlen(infile[n]) && n<=INFILEMAX) {
		n++;
	}
	
	strcpy(outfile, OUT_FILE_POS);
}
void freeInputf(char* infile[]) {
	for (int i = 0; i < INFILEMAX; i++)
	{
		free(infile[i]);
	}
}
int main()
{
	//char* optFile = "filename";
	filopt_t filopt; 
	prcopt_t prcopt;
	solopt_t solopt;
	
	int n;
	char *infile[5];
	char outfile[1024] = "";

	resetsysopts();
	getsysopts(&prcopt, &solopt, &filopt);

	initFopt(filopt);
	initPopt(prcopt);
	initSopt(solopt);
	initIn_output(infile, n, outfile);

	gtime_t ts = { 0 }, te = { 0 };
	double ti = 0.0, tu = 0.0;

	// post processing positioning
  	if (postpos(ts, te, ti, tu, &prcopt, &solopt, &filopt, infile, n, outfile, "", "") == 1)
	{
		printf("aborted");
	}

	freeInputf(infile);
    return 0;
	/**********************结构体filopt等有内存释放的bug*****************************
	-------Preliminary solution to this problem-------
	refer:	https://www.cnblogs.com/flysnail/archive/2011/09/21/2184114.html
	set:	project->配置属性->c/c++->代码生成->基本运行时检查
	value '两者(/RTC1，等同于 /RTCsu) (/RTC1)'
	to
	value '默认值'
	-------Preliminary solution to this problem-------
	-------NEED FURTHER INVESTIGATION OF THIS PROBLEM-------
	******************************************************************************/
}

/* option api
loadopts()
resetsysopts();
getsysopts();
*/