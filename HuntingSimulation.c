#include "HuntingSimulation.h"
#pragma warning(disable:6011)
//Initializing Functions:
void initializeTargetUSV(OUTPTR PTUSV* Tusv, IN Point position, IN double normalVelocity, IN double escapingVelocity, IN double alertingRadius, IN double Sradius, IN double movableAngleRange, IN double headDirection)
{
	*Tusv = (PTUSV)malloc(sizeof(TUSV));
	(*Tusv)->pos.x = position.x;
	(*Tusv)->pos.y = position.y;
	(*Tusv)->normalVelocity = normalVelocity;
	(*Tusv)->escapingVelocity = escapingVelocity;
	(*Tusv)->alertingRadius = alertingRadius;
	(*Tusv)->safetyRadius = Sradius;
	(*Tusv)->movableAngleRange = movableAngleRange;
	(*Tusv)->headDirection = headDirection;
	return;
}
void initializeHunterUsv(OUTPTR PHUSV* Husv, IN Point position, IN double velocity, IN double movableAngleRange, IN double headDirection)
{
	*Husv = (PHUSV)malloc(sizeof(HUSV));
	(*Husv)->pos.x = position.x;
	(*Husv)->pos.y = position.y;
	(*Husv)->velocity = velocity;
	(*Husv)->movableAngleRange = movableAngleRange;
	(*Husv)->headDirection = headDirection;
	(*Husv)->evaluationIndicator.nearAngleDiffer = 0.0;
	(*Husv)->evaluationIndicator.idealDistanceDiffer = 0.0;
	(*Husv)->evaluationIndicator.headDirectionDiffer = 0.0;
	(*Husv)->evaluationIndicator.idealAngleDiffer = 0.0;
	(*Husv)->hunterListEntry.Flink = NULL;
	(*Husv)->hunterListEntry.Blink = NULL;
	return;
}
void buildCycleList(IN_OUT PHUSV* hunter)
{
	for (size_t j = 0; j < huntersNum - 1; j++)
	{
		hunter[j]->hunterListEntry.Flink = &hunter[j + 1]->hunterListEntry;
	}
	hunter[huntersNum - 1]->hunterListEntry.Flink = &hunter[0]->hunterListEntry;
	for (size_t j = huntersNum - 1; j > 0; j--)
	{
		hunter[j]->hunterListEntry.Blink = &hunter[j - 1]->hunterListEntry;
	}
	hunter[0]->hunterListEntry.Blink = &hunter[huntersNum - 1]->hunterListEntry;
	return;
}
void showTarget(IN PTUSV target)
{
	printf("Target has position: (%.3f, %.3f), normalVelocity: %.3f, escapingVelocity: %.3f, dangerousRadius: %.3f, SafetyRadius: %.3f\n", target->pos.x, target->pos.y, target->normalVelocity, target->escapingVelocity, target->alertingRadius, target->safetyRadius);
	return;
}
void showHunter(IN PHUSV hunter)
{
	printf("This->hunter has position: (%.3f, %.3f) with velocity: %.3f\n", hunter->pos.x, hunter->pos.y, hunter->velocity);
	return;
}
void checkCycleListF(IN PPHUSV hunter)
{
	if (hunter)
	{
		PLIST_ENTRY temp = &((*hunter)->hunterListEntry);
		showHunter(CONTAINING_RECORD(temp, HUSV, hunterListEntry));
		while (temp->Flink != &((*hunter)->hunterListEntry))
		{
			temp = temp->Flink;
			showHunter(CONTAINING_RECORD(temp, HUSV, hunterListEntry));
		}
	}
}
void checkCycleListB(IN PPHUSV hunter)
{
	if (hunter)
	{
		PLIST_ENTRY temp = &((*hunter)->hunterListEntry);
		showHunter(CONTAINING_RECORD(temp, HUSV, hunterListEntry));
		while (temp->Blink != &((*hunter)->hunterListEntry))
		{
			temp = temp->Blink;
			showHunter(CONTAINING_RECORD(temp, HUSV, hunterListEntry));
		}
	}
}
void checkCycleList(IN PPHUSV hunter)
{
	checkCycleListF(hunter);
}
void checkEI(IN EI pEvaluationIndicator)
{
	//ckFloatValue(pevaluationIndicator.angleDiffer); 此成员隐藏，不对外暴露
	ckFloatValue(pEvaluationIndicator.idealDistanceDiffer);
	ckFloatValue(pEvaluationIndicator.headDirectionDiffer);
	ckFloatValue(pEvaluationIndicator.idealAngleDiffer);
	return;
}
//Basic Mathematical Functions:
double mabs(IN double x)
{
	return x >= 0 ? x : -x;
}
double msign(IN double x)
{
	return x >= 0 ? 1 : -1;
}
double mmin(IN double x, IN double y)
{
	return x >= y ? y : x;
}
double mmax(IN double x, IN double y)
{
	return x >= y ? x : y;
}
double cosa(IN double x)
{
	return cos(cvtAngle2Rad(x));
}
double sina(IN double x)
{
	return sin(cvtAngle2Rad(x));
}
double closeBetter(IN double indicator, IN double numberCloseTo)
{
	const double epsilon = 1e-6;
	return 1.0 / (1.0 + mabs(indicator - numberCloseTo) + epsilon);
}
BOOL floatSame10(IN double x, IN double y)
{
	if (mabs(x - y) < 0.01)
		return 1;
	else
		return 0;
}
BOOL floatSame100(IN double x, IN double y)
{
	if (mabs(x - y) < 0.001)
		return 1;
	else
		return 0;
}
BOOL floatSame1000(IN double x, IN double y)
{
	if (mabs(x - y) < 0.0001)
		return 1;
	else
		return 0;
}
//Points and Vectors Functions:
double cvtAngle2Rad(IN double x)
{
	return x * pi / 180;
}
double cvtRad2Angle(IN double x)
{
	return x * 180 / pi;
}
double calculate_V_model(IN PV v)
{
	return sqrt(v->a * v->a + v->b * v->b);
}
double calculate_V$V_mul(IN PV v1, IN PV v2)
{
	return v1->a * v2->a + v1->b * v2->b;
}
double calculate_V$V_rad_abs(IN PV v1, IN PV v2)
{
	return acos(mabs(calculate_V$V_mul(v1, v2) / (calculate_V_model(v1) * calculate_V_model(v2))));
}
double calculate_V$V_angle_abs(IN PV v1, IN PV v2)
{
	return cvtRad2Angle(calculate_V$V_rad_abs(v1, v2));
}
//calculate_OX$V_signedAngle: 根据输入向量V指向的区域判断：如果V指向第一、第二象限（上面），那么返回OX轴逆时针到V的角度，为正数【0, 180】; 如果V指向第三、第四象限（下面），那么返回OX轴顺时针到V的角度，为负数【-180,0】.
double calculate_OX$V_signedAngle(IN PV v)
{
	V OX = { 1, 0 };
	if (v->a == 0 && v->b == 0)
	{
		printf("Fatal Error: zero vector input, program exiting...\n");
		exit(0);
		return 0;
	}
	else
	{
		if (v->a == 0)
		{
			if (v->b > 0)
			{
				return 90;
			}
			else
			{
				return -90;
			}
		}
		else if (v->b == 0)
		{
			if (v->a > 0)
			{
				return 0;
			}
			else
			{
				return 180;
			}
		}
		else
		{
			if (v->a > 0 && v->b > 0)
			{
				return calculate_V$V_angle_abs(&OX, v);
			}
			else if (v->a > 0 && v->b < 0)
			{
				return calculate_V$V_angle_abs(&OX, v) * (-1);
			}
			else if (v->a < 0 && v->b > 0)
			{
				return 180 - calculate_V$V_angle_abs(&OX, v);
			}
			else
			{
				return calculate_V$V_angle_abs(&OX, v) - 180;
			}
		}
	}
}
//calculate_OX$V_unsignedAngle: 返回OX轴逆时针旋转到V的角度，为正数【0, 360】.
double calculate_OX$V_unsignedAngle(IN PV v)
{
	return calculate_OX$V_signedAngle(v) < 0 ? 360 - mabs(calculate_OX$V_signedAngle(v)) : calculate_OX$V_signedAngle(v);
}
void zoomV(OUT V* v, IN double factor)
{
	(*v).a *= factor;
	(*v).b *= factor;
	return;
}
void calculateTwoVertical_V(IN V v, OUT V* temp1, OUT V* temp2)
{
	(*temp1).a = -v.b;
	(*temp1).b = v.a;
	(*temp2).a = -(*temp1).a;
	(*temp2).b = -(*temp1).b;
	return;
}
void normalizeV(OUT V* v)
{
	zoomV(v, (1 / calculate_V_model(v)));
}
double calculate_P$P$P_unsignedAngle(IN Point A, IN Point B, IN Point C)
{
	V BA = { A.x - B.x, A.y - B.y };
	V BC = { C.x - B.x, C.y - B.y };
	double XBC = calculate_OX$V_unsignedAngle(&BC);
	double XBA = calculate_OX$V_unsignedAngle(&BA);
	if (XBC == XBA)
	{
		return 0;
	}
	else
	{
		if (XBC > XBA)
		{
			return XBC - XBA;
		}
		else
		{
			return 360 - (XBA - XBC);
		}
	}
} 
void mergeVector(IN PV vectorArray, IN size_t vectorArraySize, OUT V* vectorOut)
{
	fors(
		vectorArraySize,
		(*vectorOut).a += vectorArray[j].a;
		(*vectorOut).b += vectorArray[j].b;
	);
	return;
}
//calculate_P$P$P_unsignedAngle: 若输入A, B ,C，那么返回从A开始逆时针旋转到BC边的角度.
double calculate_P$P_distance(IN Point A, IN Point B)
{
	V v = { A.x - B.x, A.y - B.y };
	return calculate_V_model(&v);
}
//Updating Functions
void updateTargetPosByNewPoint(IN_OUT PTUSV* target, IN Point newPoint)
{
	V aheadVector = { newPoint.x - (*target)->pos.x, newPoint.y - (*target)->pos.y };
	(*target)->pos.x = newPoint.x;
	(*target)->pos.y = newPoint.y;
	(*target)->headDirection = calculate_OX$V_unsignedAngle(&aheadVector);
	return;
}
void updateHunterPosByNewPoint(IN_OUT PHUSV* hunter, IN Point newPoint)
{
	(*hunter)->pos.x = newPoint.x;
	(*hunter)->pos.y = newPoint.y;
	return;
}
void updateTargetPosByMovingDistanceAndUnsignedAngle(IN_OUT PTUSV* target, IN double movingDistance, IN double angle)
{
	(*target)->pos.x += movingDistance * cos(cvtAngle2Rad(angle));
	(*target)->pos.y += movingDistance * sin(cvtAngle2Rad(angle));
	return;
}
void updateHunterPosByMovingDistanceAndUnsignedAngle(IN_OUT PHUSV* hunter, IN double movingDistance, IN double angle)
{
	(*hunter)->pos.x += movingDistance * cos(cvtAngle2Rad(angle));
	(*hunter)->pos.y += movingDistance * sin(cvtAngle2Rad(angle));
	return;
}
void movingNormalTargetRandomly(OUT PTUSV* target)
{
	int flag = rand() % 2;
	int angle_x = (flag == 0) ? (rand() % 70 - rand() % 24) : (-1) * (rand() % 70 - rand() % 24);
	int angle_y = (flag == 0) ? (rand() % 110 - rand() % 14) : (-1) * (rand() % 110 - rand() % 14);
	(*target)->pos.x += targetNormalVelocity * flag * cosa(30);
	(*target)->pos.y += targetNormalVelocity * flag * sina(30);
	return;
}
//Geometry Functions:
void getTwoCuttingPointOnCircle(IN Point center, IN double radius, IN Point externals, OUT Point* res1, OUT Point* res2)
{
	V temp = { center.x - externals.x, center.y - externals.y };
	double step = 0.001; Point point = { 0 };
	if (calculate_V_model(&temp) < radius)
	{
		printf("Fatal Error: External point in this circle, can't get cutting point.\n");
		return;
	}
	else if (calculate_V_model(&temp) == radius)
	{
		printf("Warning: External point at the edge of this circle, returns two same points.\n");
		(*res1).x = externals.x;
		(*res1).y = externals.y;
		(*res2).x = (*res1).x;
		(*res2).y = (*res1).y;
		return;
	}
	else
	{
		for (size_t j = 0; j < (size_t)(360 / step); j++)
		{
			point.x = center.x + radius * cos(cvtAngle2Rad(j * step));
			point.y = center.y + radius * sin(cvtAngle2Rad(j * step));
			if (floatSame100(calculate_P$P$P_unsignedAngle(center, point, externals), 90) == 1)
			{
				(*res1).x = point.x;
				(*res1).y = point.y;
				break;
			}
		}
		for (size_t j = 0; j < (size_t)(360 / step); j++)
		{
			point.x = center.x + radius * cos(cvtAngle2Rad(j * step));
			point.y = center.y + radius * sin(cvtAngle2Rad(j * step));
			if (floatSame100(calculate_P$P$P_unsignedAngle(center, point, externals), 270) == 1)
			{
				(*res2).x = point.x;
				(*res2).y = point.y;
				break;
			}
		}
	}
}
BOOL checkPointIsSafe(IN Point loc, IN PTUSV target)
{
	V temp = { loc.x - target->pos.x, loc.y - target->pos.y };
	return calculate_V_model(&temp) - RtN > target->safetyRadius + RtN + Rh;
}
//Surroundding Simulation Functions:
void _SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(IN PHUSV hunter, IN PTUSV target)
{
	//mostSafetyRadius是目标的安全圈半径加上一个采样时间内目标能够移动的长度；
	//RtN和Rh的系数必须大于等于1才能确保最糟糕的情况下（捕食者和目标相向而行）目标也不会发现捕食者.
	double mostSafetyRadius = target->safetyRadius + 1 * RtN + 1 * Rh;
	PHUSV curr = hunter;
	PHUSV next = CONTAINING_RECORD(curr->hunterListEntry.Flink, HUSV, hunterListEntry);
	PHUSV prev = CONTAINING_RECORD(curr->hunterListEntry.Blink, HUSV, hunterListEntry);
	hunter->evaluationIndicator.nearAngleDiffer = calculate_P$P$P_unsignedAngle(curr->pos, target->pos, next->pos) - calculate_P$P$P_unsignedAngle(prev->pos, target->pos, curr->pos);
	hunter->evaluationIndicator.headDirectionDiffer = mabs(curr->headDirection - target->headDirection);
	hunter->evaluationIndicator.idealAngleDiffer = mabs(calculate_P$P$P_unsignedAngle(curr->pos, target->pos, next->pos)) - ((double)360 / (double)huntersNum);
	hunter->evaluationIndicator.idealDistanceDiffer = calculate_P$P_distance(curr->pos, target->pos) - mostSafetyRadius;
	return;
}
double _SURROUNDING_getRewardPointsForHunterByHuntersEI(IN PHUSV hunter)
{
	EI abs_hunterEI = { 0 };

	abs_hunterEI.idealAngleDiffer = mabs(hunter->evaluationIndicator.idealAngleDiffer);
	abs_hunterEI.idealDistanceDiffer = mabs(hunter->evaluationIndicator.idealDistanceDiffer);

	double J = 0.0;
	//误差线episilon_angle:以目标为中心的合围图形中心角越接近【360 / n + episilon_angle】越好
	double episilon_angle = 2.0;
	//误差线episilon，捕食者和目标最大安全圈的距离（mostSafetyRadius）越接近0.1越好；
	//注：这里只能接受mostSafetyRadius + 0.1, 如果捕食者进入了最大安全圈内部直接拒绝进入.
	double episilon_distance = 0.2;

	if (hunter->evaluationIndicator.idealDistanceDiffer <= 0)
	{
		J = -10000.0;
	}
	else
	{
		J = w_idealAngleDiffer * closeBetter(abs_hunterEI.idealAngleDiffer, episilon_angle) + w_distanceDiffer * closeBetter(abs_hunterEI.idealDistanceDiffer, episilon_distance);
	}
	
	return J;
}
void _SURROUNDING_changingHunterInfomationByRewardFunction(IN_OUT PHUSV* hunter, IN PTUSV target)
{
	Point nextHunterIdealLoc = { 0 };
	PHUSV nextHunterIdealInfo = NULL;

	initializeHunterUsv(&nextHunterIdealInfo, nextHunterIdealLoc, 0, 0, 0);
	memcpy(nextHunterIdealInfo, *hunter, sizeof(HUSV));

	double newMaxRewardPoints = -9999.0;

	int global_j = 0;
	int global_i = 0;

	for (int j = 0; j < 180; j++)
	{
		for (int i = 1; i <= 20; i++)
		{
			nextHunterIdealInfo->pos.x = (*hunter)->pos.x + (i * ((double)huntersVelocity / (double)20.0)) * cosa(j * 2);
			nextHunterIdealInfo->pos.y = (*hunter)->pos.y + (i * ((double)huntersVelocity / (double)20.0)) * sina(j * 2);
			_SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(nextHunterIdealInfo, target);
			//必须先得到EI才能进行评分，所以要有getHuntersFourEIByHuntersLocAndHeadDirection
			//<<<<<此循环不会对*hunter造成副作用>>>>>
			if (_SURROUNDING_getRewardPointsForHunterByHuntersEI(nextHunterIdealInfo) > newMaxRewardPoints)
			{
				//newMaxRewardPoints是最大值判定变量，每当有新的最大值就更新此newMaxRewardPoints.
				newMaxRewardPoints = _SURROUNDING_getRewardPointsForHunterByHuntersEI(nextHunterIdealInfo);
				//此循环的最终目的只是输出global_ij来确定*hunter的下一个最优位置.
				global_j = j;
				global_i = i;
			}
		}
	}

	nextHunterIdealLoc.x = (*hunter)->pos.x + (global_i * ((double)huntersVelocity / (double)20.0)) * cosa(global_j * 2);
	nextHunterIdealLoc.y = (*hunter)->pos.y + (global_i * ((double)huntersVelocity / (double)20.0)) * sina(global_j * 2);

	double dis[2] = { 0.0 };
	double huntersSafetyNeighborDistance = 1.2 * Rh;
	
	nextHunterIdealInfo->pos.x = nextHunterIdealLoc.x;
	nextHunterIdealInfo->pos.y = nextHunterIdealLoc.y;

	//分布式控制：只需要最近的两个船的信息来确保安全性；
	//如果下一次移动的理想位置会导致两个捕食者距离过近，那么就让这个船停滞不动。

	dis[0] = calculate_P$P_distance(CONTAINING_RECORD(nextHunterIdealInfo->hunterListEntry.Flink, HUSV, hunterListEntry)->pos, nextHunterIdealInfo->pos);
	dis[1] = calculate_P$P_distance(CONTAINING_RECORD(nextHunterIdealInfo->hunterListEntry.Blink, HUSV, hunterListEntry)->pos, nextHunterIdealInfo->pos);
	
	if (dis[0] <= huntersSafetyNeighborDistance || dis[1] <= huntersSafetyNeighborDistance)
	{
		free(nextHunterIdealInfo);
		nextHunterIdealInfo = NULL;
		return;
	}
	//获得了最优global_ij后，先更新*hunter的位置，再根据*hunter新的位置去更新*hunter的EI；
	//对输入参数*hunter的副作用仅在此处实现；
	//根据获得的最优global_ij来转换成新的*hunter最优坐标，依新的最优坐标来更新*hunter的EI.
	(*hunter)->pos.x = nextHunterIdealLoc.x;
	(*hunter)->pos.y = nextHunterIdealLoc.y;
	_SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(*hunter, target);
	free(nextHunterIdealInfo);
	nextHunterIdealInfo = NULL;
	return;
}
BOOLEAN isSurroundingSuccess(IN PHUSV hunter[huntersNum], IN PTUSV target)
{
	//maxTolerance这个值是临界值；
	//如果有任何一个捕食者到目标的距离大于mostSafetyRadius加上这个maxTolerance值，那么就视为不收敛；
	//maxTolerance值必须大于_SURROUNDING_getRewardPointsForHunterByHuntersEI中定义的episilon_distance；
	//否则永不收敛

	double maxDistanceTolerance = 0.3;
	double maxAngleTolerance = 2.0;

	double d[huntersNum] = { 0 };
	double a[huntersNum] = { 0 };

	for (size_t j = 0; j < huntersNum; j++)
	{
		d[j] = hunter[j]->evaluationIndicator.idealDistanceDiffer;
		a[j] = hunter[j]->evaluationIndicator.idealAngleDiffer;
	}
	for (size_t j = 0; j < huntersNum; j++)
	{
		if (d[j] <= 0.0 || d[j] >= maxDistanceTolerance)
		{
			return 0;
		}
		if (mabs(a[j]) >= maxAngleTolerance)
		{
			return 0;
		}
	}
	
	return 1;
	
}
//Alerting Target Functions:
static void _ALERTING_alertingTargetIntensionally(IN PHUSV hunter[huntersNum], IN PTUSV target)
{
	double minDistance = 9999999.0;
	size_t global_j = 0;
	//选择最近的捕食者：
	fors(
		huntersNum,
		if (hunter[j]->evaluationIndicator.idealDistanceDiffer < minDistance)
		{
			minDistance = hunter[j]->evaluationIndicator.idealDistanceDiffer;
			global_j = j;
		}
	);
	//确定最近的捕食者的移动方向，直接向目标移动：
	V closestHunterMovingDirection = { 0 };
	closestHunterMovingDirection.a = target->pos.x - hunter[global_j]->pos.x;
	closestHunterMovingDirection.b = target->pos.y - hunter[global_j]->pos.y;
	//确定最终的移动角度：
	double ultimateMovingAngle = calculate_OX$V_unsignedAngle(&closestHunterMovingDirection);
	//移动最近的捕食者，直接向目标移动：
	updateHunterPosByMovingDistanceAndUnsignedAngle(&hunter[global_j], Rh, ultimateMovingAngle);
	//调试信息打印：
	ckFloatValue(calculate_P$P_distance(hunter[global_j]->pos, target->pos));

	return;
}
//TestRoutine: all hunters go towards target directly.
void testRoutine(IN PHUSV hunter[huntersNum], IN PTUSV target)
{
	V direction[huntersNum] = { {0} };
	fors(
		huntersNum,
		direction[j].a = target->pos.x - hunter[j]->pos.x;
		direction[j].b = target->pos.y - hunter[j]->pos.y;
	);
	double angle[huntersNum] = { 0.0 };
	fors(
		huntersNum,
		angle[j] = calculate_OX$V_unsignedAngle(&direction[j]);
	);
	fors(
		huntersNum,
		updateHunterPosByMovingDistanceAndUnsignedAngle(&hunter[j], Rh, angle[j]);
	);
	return;
}
//Target Escaping Simulation Functions:
double getEscapingAngleByVectorMethod(IN PHUSV hunter[huntersNum], IN PTUSV target)
{
	//进入了目标危险半径内的捕食者个数：
	size_t huntersInDangerousRadius = 0;
	//进入了目标危险半径内捕食者的指标数组：若第j个捕食者进入了危险半径内则其对应的index为1，否则为0：
	int huntersInDangerousFlagIndex[huntersNum] = { 0 };
	//确定进入了目标危险半径内的捕食者个数和指标数组：
	fors(
		huntersNum,
		if (calculate_P$P_distance(hunter[j]->pos, target->pos) < target->alertingRadius)
		{
			huntersInDangerousRadius++;
			huntersInDangerousFlagIndex[j]++;
		}
	);
	//分配内存，仅在危险半径内的目标-捕食者向量才会被计算：
	PV dangerousVector = (PV)malloc(huntersInDangerousRadius * sizeof(V));
	//零初始化：
	RtlZeroMemory(dangerousVector, huntersInDangerousRadius * sizeof(V));
	//由于使用了FLAG_ARRAY-VALID_COUNT范式，用自增index方法对malloc的内存进行顺序分配：
	size_t index = 0;
	//如果分配成功（几乎100%）
	if (dangerousVector)
	{
		for (size_t j = 0; j < huntersNum; j++)
		{
			if (huntersInDangerousFlagIndex[j] == 1)
			{
				//如果第j个捕食者处于危险半径内，那么赋值由malloc分配的向量：
				dangerousVector[index].a = target->pos.x - hunter[j]->pos.x;
				dangerousVector[index].b = target->pos.y - hunter[j]->pos.y;
				//自增index来循环
				index++;
				//防止溢出：
				if (index > huntersInDangerousRadius)
				{
					break;
				}
			}
		}
	}
	//计算最终的方向：
	V ultimateVector = { 0 };
	mergeVector(dangerousVector, huntersInDangerousRadius, &ultimateVector);
	//确定最终移动角度
	double ultimateMovingAngle = calculate_OX$V_unsignedAngle(&ultimateVector);
	//释放malloc内存：
	free(dangerousVector);
	dangerousVector = NULL;
	return ultimateMovingAngle;
}
BOOLEAN isRecyclingSuccess(PHUSV hunter[huntersNum], PTUSV target)
{
	double d[huntersNum] = { 0 };
	double a[huntersNum] = { 0 };
	for (size_t j = 0; j < huntersNum; j++)
	{
		d[j] = hunter[j]->evaluationIndicator.idealDistanceDiffer;
		a[j] = hunter[j]->evaluationIndicator.idealAngleDiffer;
	}
	for (size_t j = 0; j < huntersNum; j++)
	{
		if (d[j] <= 0 || d[j] >= 3)
		{
			return 0;
		}
		if (mabs(a[j]) >= 2)
		{
			return 0;
		}
	}

	return 1;

}
//C-Python Interaction Functions:
void makePythonFile(FILE* fp, PHUSV hunter[3], PTUSV target) 
{
	Point h[3] = { {0} };

	for (int j = 0; j < huntersNum; j++)
	{
		h[j].x = hunter[j]->pos.x;
		h[j].y = hunter[j]->pos.y;
	}

	Point t = { 0 };

	t.x = target->pos.x;
	t.y = target->pos.y;

	if (fp == NULL) 
	{
		fprintf(stderr, "File pointer is NULL.\n");
		return;
	}

	for (int i = 0; i < 3; i++) 
	{
		fprintf(fp, "hunter %d: x = %.3f, hunter %d: y = %.3f\n", i, h[i].x, i, h[i].y);
	}

	fprintf(fp, "target: x = %.3f, target: y = %.3f\n", t.x, t.y);

	fprintf(fp, "QAQ\n");

	return;
}
