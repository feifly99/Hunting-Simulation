#ifndef __HuntingSimulation__
#define __HuntingSimulation__

#include "MathWorks.h"
#include "colorPrint.h"

#define QAQ printf("\n")

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifndef IN_OUT
#define IN_OUT
#endif //always stay with P-type, means may operate and change pointer directly.

#ifndef OUTPTR
#define OUTPTR
#endif //means out ptr is created by MALLOC.

#define fors(times, sentence) \
do\
{\
	for(size_t j = 0; j < times; j++) \
	{\
		sentence\
	}\
}while (0)

#define ckFloatValue(sentence) \
do\
{\
	printf("%s = %lf\n", (const char*)#sentence, (float)(sentence));\
}while (0)

#define ckIntValue(sentence) \
do\
{\
	printf("%s = %ld\n", (const char*)#sentence, (int)(sentence));\
}while (0)

#define ckSizeTValue(sentence) \
do\
{\
	printf("%s = %zu\n", (const char*)#sentence, (size_t)(sentence));\
}while (0)

#define bug(num) printf("%ld\n", num)

#define targetNormalVelocity 3.0
#define targetSafetyRadius 8000

#define huntersNum 300
#define huntersVelocity (0.5 * (double)pi * (double)targetNormalVelocity)
#define huntersSafetyRadius (huntersVelocity * 1.5)

#define obstaclesNum 5000
#define obstacleSafetyFactor 2.0

#define RtN (targetNormalVelocity)
#define Rh (huntersVelocity)

#define ANGLE_REGION_SIZE 360
#define DIS_REGION_SIZE 20
#define OFFSETS_MAX_SIZE (ANGLE_REGION_SIZE * DIS_REGION_SIZE + 1)

#define PHASE_ANGLE (360.0 / (double)huntersNum)

typedef struct _Point
{
	double x;
	double y;
}Point, * PPoint, ** PPPoint;
typedef struct _vector
{
	double a;
	double b;
}V, * PV, ** PPV;
typedef struct _targetUSV
{
	Point pos;
	double normalVelocity;
	double safetyRadius;
}TUSV, * PTUSV, ** PPTUSV;
typedef struct _huntingUSV
{
	Point pos;
	size_t hunterListIndex;
	double velocity;
	LIST_ENTRY hunterListEntry; //后置定义
	size_t deadCorner;
}HUSV, * PHUSV, ** PPHUSV; //后置定义在第一次初始化时没有特定值，需要在后续的功能函数单独赋值。
typedef struct _OBSTACLE
{
	Point center;
	double size;
	double v;
	double movingDirection;
}OBSTACLE, * POBSTACLE;
//Initializing Functions:
void initializeTargetUSV(OUTPTR PTUSV* Tusv, IN Point position, IN double normalVelocity, IN double safetyRadius);
void initializeHunterUsv(OUTPTR PHUSV* Husv, IN Point position, IN double velocity, IN size_t hunterListIndex);
void initializeObstacles(OUTPTR POBSTACLE* obstacle, IN Point center, IN double size, IN double velocity, IN double movingDirection);
void buildCycleListByHuntersOriginalIndex(IN_OUT PHUSV* hunter);
void showTarget(IN PTUSV target);
void showHunter(IN PHUSV hunter);
void showObstacle(IN POBSTACLE obstacle);
void checkHuntersCycleListF(IN PPHUSV hunter);
void checkHuntersCycleListB(IN PPHUSV hunter);
void checkHuntersCycleList(IN PPHUSV hunter);
//Mathematical Functions:
double mabs(IN double x);
double msign(IN double x);
double mmin(IN double x, IN double y);
double mmax(IN double x, IN double y);
double cosa(IN double x);
double sina(IN double x);
double closeBetter(IN double indicator, IN double numberCloseTo);
void sortArray(IN double* array, IN size_t size);
BOOL floatSame10(IN double x, IN double y);
BOOL floatSame100(IN double x, IN double y);
BOOL floatSame1000(IN double x, IN double y);
double cvtAngle2Rad(IN double x);
double cvtRad2Angle(IN double x);
//Points and Vectors Functions:
double calculate_V_model(IN PV v);
double calculate_V$V_mul(IN PV v1, IN PV v2);
double calculate_V$V_rad_abs(IN PV v1, IN PV v2);
double calculate_V$V_angle_abs(IN PV v1, IN PV v2);
double calculate_OX$V_signedAngle(IN PV v);
double calculate_OX$V_unsignedAngle(IN PV v);
void zoomV(OUT V* v, IN double factor);
void calculateTwoVertical_V(IN V v, OUT V* temp1, OUT V* temp2);
void normalizeV(OUT V* v);
double calculate_P$P$P_unsignedAngle(IN Point A, IN Point B, IN Point C);
void mergeVector(IN PV vectorArray, IN size_t vectorArraySize, OUT V* vectorOut);
double calculate_P$P_distance(IN Point A, IN Point B);
//Updating Functions:
void updateTargetPosByNewPoint(IN_OUT PTUSV* target, IN Point newPoint);
void updateHunterPosByNewPoint(IN_OUT PHUSV* hunter, IN Point newPoint);
void updateTargetPosByMovingDistanceAndUnsignedAngle(IN_OUT PTUSV* target, IN double movingDistance, IN double angle);
void updateHunterPosByMovingDistanceAndUnsignedAngle(IN_OUT PHUSV* hunter, IN double movingDistance, IN double angle);
void movingNormalTargetRandomly(OUT PTUSV* target);
void movingNormalTargetStably(OUT PTUSV* target);
void movingObstaclesRandomly(OUT POBSTACLE** obstacles);
//Geometry Functions:
void getTwoCuttingPointOnCircle(IN Point center, IN double radius, IN Point externals, OUT Point* res1, OUT Point* res2);
BOOL checkPointIsSafe(IN Point loc, IN PTUSV target);
//Surroundding Simulation Functions:
void _SURROUNDING_changingHunterInfomationByRewardFunctionAndEnvironment(IN_OUT PHUSV* hunter, IN PTUSV target, IN POBSTACLE obstacles[obstaclesNum]);
BOOLEAN isSurroundingSuccess(IN PHUSV hunter[huntersNum], IN PTUSV target, IN POBSTACLE* obstacles);
//C-Python Interaction Functions:
void makePythonFile(IN FILE* fp, IN PHUSV hunter[huntersNum], IN PTUSV target, IN POBSTACLE obstacles[obstaclesNum]);

#endif
