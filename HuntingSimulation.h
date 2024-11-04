#ifndef __HuntingSimulation__
#define __HuntingSimulation__
#include "MathWorks.h"

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
}while (0); 

#define ckFloatValue(sentence) \
do\
{\
	printf("%s = %lf\n", (const char*)#sentence, sentence);\
}while (0)

#define ckIntValue(sentence) \
do\
{\
	printf("%s = %ld\n", (const char*)#sentence, sentence);\
}while (0)

#define ckSizeTValue(sentence) \
do\
{\
	printf("%s = %zu\n", (const char*)#sentence, sentence);\
}while (0)

#define bug(num) printf("%ld\n", num)

#define samplingTimeGap 1 
#define MaxRunningAngle 80

#define targetNormalVelocity 1.5
#define targetEscapingVelocity 3
#define targetAlertingRadius 7
#define targetSafetyRadius 8
#define targetMovableAngleRange 60

#define huntersNum 5
#define huntersVelocity (0.6 * pi * targetNormalVelocity * samplingTimeGap)
#define huntersSafetyRadius 8
#define huntersRecycleRadius 2
#define huntersMovableAngleRange 80

#define SEMI_SURROUNDING_ANGLE_MIN 90
#define SEMI_SURROUNDING_ANGLE_MAX 140

#define RtN (targetNormalVelocity * samplingTimeGap)
#define RtE (1.5 * huntersVelocity)
#define Rh (huntersVelocity * samplingTimeGap)

#define w_hidden 0
#define w_headDirectionDiffer 0.1
#define w_idealAngleDiffer 11
#define w_distanceDiffer 3

#define huntersRecyclingRadius 3
#define targetHarmingRadius 2

#define targetEscapingWeights_angle 5
#define targetEscapingWeights_distance 20

#define STACK_MAX_SIZE 600

typedef struct _evaluationIndicators
{
	double nearAngleDiffer; //三个相互邻接的，以目标USV为中心的HUSV的夹角之差
	//备注：nearAngleDiffer隐藏于结构体中，不对外暴露
	double idealDistanceDiffer; //当前HUSV和目标安全圈的距离
	double headDirectionDiffer; //当前HUSV的朝向和目标朝向角度之差
	double idealAngleDiffer; //当前HUSV，目标USV和它的nextHUSV的夹角与等分角的差值
}EI, * PEI, ** PPEI;
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
	double escapingVelocity;
	double alertingRadius;
	double safetyRadius;
	double movableAngleRange;
	double headDirection;
}TUSV, * PTUSV, ** PPTUSV;
typedef struct _huntingUSV
{
	Point pos;
	size_t originalIndex;
	size_t currentIndex;
	double velocity;
	double movableAngleRange;
	double headDirection;
	BOOLEAN isInsightByTarget;
	EI evaluationIndicator; //后置定义
	LIST_ENTRY hunterListEntry; //后置定义
}HUSV, * PHUSV, ** PPHUSV; //后置定义在第一次初始化时没有特定值，需要在后续的功能函数单独赋值。
//Initializing Functions:
void initializeTargetUSV(OUTPTR PTUSV* Tusv, IN Point position, IN double normalVelocity, IN double escapingVelocity, IN double alertingRadius, IN double Sradius, IN double movableAngleRange, IN double headDirection);
void initializeHunterUsv(OUTPTR PHUSV* Husv, IN Point position, IN double velocity, IN double movableAngleRange, IN double headDirection, IN size_t originalIndex);
void buildCycleListByHuntersOriginalIndex(IN_OUT PHUSV* hunter);
void showTarget(IN PTUSV target);
void showHunter(IN PHUSV hunter);
void checkHuntersCycleListF(IN PPHUSV hunter);
void checkHuntersCycleListB(IN PPHUSV hunter);
void checkHuntersCycleList(IN PPHUSV hunter);
void checkHuntersEI(IN EI evaluationIndicator);
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
double calculate_P$P_distance(IN Point A, IN Point B);
void mergeVector(IN PV vectorArray, IN size_t vectorArraySize, OUT V* vectorOut);
//Updating Functions:
void updateTargetPosByNewPoint(IN_OUT PTUSV* target, IN Point newPoint);
void updateHunterPosByNewPoint(IN_OUT PHUSV* hunter, IN Point newPoint);
void updateTargetPosByMovingDistanceAndUnsignedAngle(IN_OUT PTUSV* target, IN double movingDistance, IN double angle);
void updateHunterPosByMovingDistanceAndUnsignedAngle(IN_OUT PHUSV* hunter, IN double movingDistance, IN double angle);
void movingNormalTargetRandomly(OUT PTUSV* target);
void movingNormalTargetStably(OUT PTUSV* target);
//Geometry Functions:
void getTwoCuttingPointOnCircle(IN Point center, IN double radius, IN Point externals, OUT Point* res1, OUT Point* res2);
BOOL checkPointIsSafe(IN Point loc, IN PTUSV target);
//Surroundding Simulation Functions:
void _SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(IN_OUT PHUSV* hunter, IN PTUSV target);
double _SURROUNDING_getRewardPointsForHunterByHuntersEI(IN PHUSV hunter);
void _SURROUNDING_changingHunterInfomationByRewardFunction(IN_OUT PHUSV* hunter, IN PTUSV target);
void _SURROUNDING_remakeHuntersCycleListByRelativeLocCompareToTarget(IN_OUT PHUSV (*hunter)[huntersNum], IN PTUSV target);
BOOLEAN isSurroundingSuccess(IN PHUSV hunter[huntersNum], IN PTUSV target);
//Hunters Hunting Method Simulation Functions:
void _HUNTER_goTowardsTargetDirectly(IN_OUT PHUSV* hunter, IN PTUSV target);
//Target Escaping Simulation Functions:
void _TARGET_escapingAngleByVectorMethod(IN PHUSV hunter[huntersNum], IN PTUSV* target);
void _TARGET_escapingAngleByLearningMethod(IN PHUSV hunter[huntersNum], IN PTUSV* target);
//Recycling Outcomes Judge Functions:
BOOLEAN isRecyclingSuccess(IN PHUSV hunter[huntersNum], IN PTUSV target);
BOOLEAN isTerminatingRecycling(IN PHUSV hunter[huntersNum], IN PTUSV target);
//void _TARGET_escapingAngleByLearningMethod(IN PHUSV hunter[huntersNum], IN PTUSV* target);
//C-Python Interaction Functions:
void makePythonFile(IN FILE* fp, IN PHUSV hunter[huntersNum], IN PTUSV target);
#endif
