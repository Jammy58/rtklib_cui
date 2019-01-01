#include "stdafx.h"
#include "rtklib.h"

#define MAX_FILE_NAME 1024
#define STATFILE    "rtknavi_%Y%m%d%h%M.stat"  // solution status file
#define TRACEFILE   "rtknavi_%Y%m%d%h%M.trace" // debug trace file
//123.56.239.141 4023
//ntrip.gnsslab.cn 2101
rtksvr_t rtksvr;                        // rtk server struct
prcopt_t PrcOpt;
solopt_t SolOpt;
filopt_t filopt;
char*a = "@\n";

int RovPosTypeF, RefPosTypeF, RovAntPcvF, RefAntPcvF;
double RovPos[3], RefPos[3];
double RefAntDel[3] = { 0 }, RovAntDel[3] = { 0 }; // 天线高偏移量
char RovAntF[MAX_FILE_NAME] = { 0 };   // 天线类型名称
char RefAntF[MAX_FILE_NAME] = { 0 }; 
char SatPcvFileF[MAX_FILE_NAME] = { 0 }; // .atx文件路径
char AntPcvFileF[MAX_FILE_NAME] = { 0 };
char GeoidDataFileF[MAX_FILE_NAME] = { 0 };
char DCBFileF[MAX_FILE_NAME] = { 0 };
int Stream[MAXSTRRTK], StreamC[MAXSTRRTK], Format[MAXSTRRTK]; // input stream info
char *Paths[MAXSTRRTK][4], *Cmds[3][2], *CmdsTcp[3][2];
int CmdEna[3][2], CmdEnaTcp[3][2];
int NmeaCycle;
double NmeaPos[2];
char RcvOpt[3][MAX_FILE_NAME];
char LocalDirectory[] = "C:\Temp";
int DebugTraceF = 0, DebugStatusF=0;
int TimeoutTime = 10000, ReconTime = 10000;
int SvrBuffSize = 32768;
int FileSwapMargin = 30;
int SvrCycle = 10, NavSelect=0, NmeaReq=0;
stream_t monistr;                       // monitor stream
int PSol, PSolS, PSolE, Nsat[2];
int *SolStat, *Nvsat;
gtime_t *Time;
double *SolRov, *SolRef, *Qr, *VelRov, *Age, *Ratio;
int SolBuffSize= 1000;

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
void  InitSolBuff(void)
{
	double ep[] = { 2000,1,1,0,0,0 };
	int i, j;

	trace(3, "InitSolBuff\n");

	delete[] Time;   delete[] SolStat; delete[] Nvsat;  delete[] SolRov;
	delete[] SolRef; delete[] Qr;      delete[] VelRov; delete[] Age;
	delete[] Ratio;

	if (SolBuffSize <= 0) SolBuffSize = 1;
	Time = new gtime_t[SolBuffSize];
	SolStat = new int[SolBuffSize];
	Nvsat = new int[SolBuffSize];
	SolRov = new double[SolBuffSize * 3];
	SolRef = new double[SolBuffSize * 3];
	VelRov = new double[SolBuffSize * 3];
	Qr = new double[SolBuffSize * 9];
	Age = new double[SolBuffSize];
	Ratio = new double[SolBuffSize];
	PSol = PSolS = PSolE = 0;
	for (i = 0; i < SolBuffSize; i++) {
		Time[i] = epoch2time(ep);
		SolStat[i] = Nvsat[i] = 0;
		for (j = 0; j < 3; j++) SolRov[j + i * 3] = SolRef[j + i * 3] = VelRov[j + i * 3] = 0.0;
		for (j = 0; j < 9; j++) Qr[j + i * 9] = 0.0;
		Age[i] = Ratio[i] = 0.0;
	}
}

void init() {
	rtksvrinit(&rtksvr);
	strinit(&monistr);
	resetsysopts();
	getsysopts(&PrcOpt, &SolOpt, &filopt);
	initPopt(PrcOpt);

	RovPosTypeF = RefPosTypeF = 0;
	RovPos[0] = RovPos[1] = RovPos[2] = 0.0;
	RefPos[0] = RefPos[1] = RefPos[2] = 0.0;

}

extern void rtksrv_cui_start() {
	using namespace std;
	solopt_t solopt[2];
	double pos[3], nmeapos[3];
	int itype[] = { STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPCLI,STR_FILE,STR_FTP,STR_HTTP };
	int otype[] = { STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPSVR,STR_FILE };
	int i, strs[MAXSTRRTK] = { 0 }, sat, ex, stropt[8] = { 0 };
	char *paths[8], *cmds[3] = { 0 }, *rcvopts[3] = { 0 };
	char buff[1024], *p;
	/* qt build debug */
	char* cmds_periodic[3] = { NULL,NULL,NULL };
	char errmsg[20] = { 0 };

	gtime_t time = timeget();
	pcvs_t pcvr, pcvs;
	pcv_t *pcv;

	trace(3, "SvrStart\n");

	memset(&pcvr, 0, sizeof(pcvs_t));
	memset(&pcvs, 0, sizeof(pcvs_t));

	if (RovPosTypeF <= 2) { // LLH,XYZ
		PrcOpt.rovpos = 0;
		PrcOpt.ru[0] = RovPos[0];
		PrcOpt.ru[1] = RovPos[1];
		PrcOpt.ru[2] = RovPos[2];
	}
	else { // RTCM position
		PrcOpt.rovpos = 4;
		for (i = 0; i < 3; i++) PrcOpt.ru[i] = 0.0;
	}
	if (RefPosTypeF <= 2) { // LLH,XYZ
		PrcOpt.refpos = 0;
		PrcOpt.rb[0] = RefPos[0];
		PrcOpt.rb[1] = RefPos[1];
		PrcOpt.rb[2] = RefPos[2];
	}
	else if (RefPosTypeF == 3) { // RTCM position
		PrcOpt.refpos = 4;
		for (i = 0; i < 3; i++) PrcOpt.rb[i] = 0.0;
	}
	else { // average of single position
		PrcOpt.refpos = 1;
		for (i = 0; i < 3; i++) PrcOpt.rb[i] = 0.0;
	}

	for (i = 0; i < MAXSAT; i++) {
		PrcOpt.exsats[i] = 0;
	}
	
	/* exclude sat :todo */

	if ((RovAntPcvF || RefAntPcvF) && !readpcv(AntPcvFileF, &pcvr)) {
		cout << "rcv ant file read error" << AntPcvFileF << endl;
		return;
	}
	if (RovAntPcvF) {
		if ((pcv = searchpcv(0, RovAntF, time, &pcvr))) {
			PrcOpt.pcvr[0] = *pcv;
		}
		else {
			cout << "no antenna pcv " << RovAntF << endl;
		}
		for (i = 0; i < 3; i++) PrcOpt.antdel[0][i] = RovAntDel[i];
	}
	if (RefAntPcvF) {
		if ((pcv = searchpcv(0, RefAntF, time, &pcvr))) {
			PrcOpt.pcvr[1] = *pcv;
		}
		else {
			cout << "no antenna pcv" << RefAntF << endl;
		}
		for (i = 0; i < 3; i++) PrcOpt.antdel[1][i] = RefAntDel[i];
	}
	if (RovAntPcvF || RefAntPcvF) {
		free(pcvr.pcv);
	}
	if (PrcOpt.sateph == EPHOPT_PREC || PrcOpt.sateph == EPHOPT_SSRCOM) { // 卫星质心则需进行天线类型改正
		if (!readpcv(SatPcvFileF, &pcvs)) {
			cout << "sat ant file read error" << SatPcvFileF << endl;
			return;
		}
		for (i = 0; i < MAXSAT; i++) {
			if (!(pcv = searchpcv(i + 1, "", time, &pcvs))) continue;
			rtksvr.nav.pcvs[i] = *pcv;
		}
		free(pcvs.pcv);
	}

	for (i = 0; i < 3; i++) strs[i] = StreamC[i] ? itype[Stream[i]] : STR_NONE;
	for (i = 3; i < 5; i++) strs[i] = StreamC[i] ? otype[Stream[i]] : STR_NONE;
	for (i = 5; i < 8; i++) strs[i] = StreamC[i] ? otype[Stream[i]] : STR_NONE;
	for (i = 0; i < 8; i++) {
		paths[i] = new char[1024];
		paths[i][0] = '\0';
		if (strs[i] == STR_NONE) strcpy(paths[i], "");
		else if (strs[i] == STR_SERIAL) strcpy(paths[i], Paths[i][0]);
		else if (strs[i] == STR_FILE) strcpy(paths[i], Paths[i][2]);
		else if (strs[i] == STR_FTP || strs[i] == STR_HTTP) strcpy(paths[i], Paths[i][3]);
		else strcpy(paths[i], Paths[i][1]);
	}
	for (i = 0; i < 3; i++) {
		cmds[i] = new char[1024];
		rcvopts[i] = new char[1024];
		cmds[i][0] = rcvopts[i][0] = '\0';
		if (strs[i] == STR_SERIAL) {
			if (CmdEna[i][0]) strcpy(cmds[i], Cmds[i][0]);
		}
		else if (strs[i] == STR_TCPCLI || strs[i] == STR_TCPSVR ||
			strs[i] == STR_NTRIPCLI) {
			if (CmdEnaTcp[i][0]) strcpy(cmds[i], CmdsTcp[i][0]);
		}
		strcpy(rcvopts[i], RcvOpt[i]);
	}
	NmeaCycle = NmeaCycle < 1000 ? 1000 : NmeaCycle;
	pos[0] = NmeaPos[0] * D2R;
	pos[1] = NmeaPos[1] * D2R;
	pos[2] = 0.0;
	pos2ecef(pos, nmeapos);

	strsetdir(LocalDirectory);
	strsetproxy("");


	if (DebugTraceF > 0) {
		traceopen(TRACEFILE);
		tracelevel(DebugTraceF);
	}
	if (DebugStatusF > 0) {
		rtkopenstat(STATFILE, DebugStatusF);
	}
	if (SolOpt.geoid > 0 && GeoidDataFileF != "") {
		opengeoid(SolOpt.geoid, GeoidDataFileF);
	}
	if (DCBFileF != "") {
		readdcb(DCBFileF, &rtksvr.nav, NULL);
	}
	for (i = 0; i < 2; i++) {
		solopt[i] = SolOpt;
		solopt[i].posf = Format[i + 3];
	}
	
	stropt[0] = TimeoutTime;
	stropt[1] = ReconTime;
	stropt[2] = 1000;
	stropt[3] = SvrBuffSize;
	stropt[4] = FileSwapMargin;
	strsetopt(stropt);

	// start rtk server
	if (!rtksvrstart(&rtksvr, SvrCycle, SvrBuffSize, strs,
		paths, Format, NavSelect, cmds,
		cmds_periodic, rcvopts, NmeaCycle,
		NmeaReq, nmeapos, &PrcOpt, solopt,
		&monistr, errmsg)) {
		traceclose();
		for (i = 0; i < 8; i++) delete[] paths[i];
		for (i = 0; i < 3; i++) delete[] rcvopts[i];
		for (i = 0; i < 3; i++) delete[] cmds[i];
		return;
	}
	for (i = 0; i < 8; i++) delete[] paths[i];
	for (i = 0; i < 3; i++) delete[] rcvopts[i];
	for (i = 0; i < 3; i++) delete[] cmds[i];
	PSol = PSolS = PSolE = 0;
	SolStat[0] = Nvsat[0] = 0;
	for (i = 0; i < 3; i++) SolRov[i] = SolRef[i] = VelRov[i] = 0.0;
	for (i = 0; i < 9; i++) Qr[i] = 0.0;
	Age[0] = Ratio[0] = 0.0;
	Nsat[0] = Nsat[1] = 0;
}