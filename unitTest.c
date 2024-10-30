#include "HuntingSimulation.h"

int main()
{
	srand((UINT)time(NULL));
	FILE* fp = fopen("output.txt", "w");
	PTUSV target = NULL;
	//�������Ŀ������꣬��ע�Ⲷʳ�ߵ���������˳��.
	Point targetPos = { -16, 9 };
	PHUSV hunter[huntersNum] = { NULL };
	//��Ŀ��������Ҳ����߷�����ʱ����ת����һ�������Ĳ�ʳ��Ϊ����ͷ�����һ��������Ϊ����β.
	Point hunterPos[huntersNum] = { {7, 18}, {3, 2},{1, 8} };
	double hunterHeadDirection[3] = { 30, 45, 60 };
	EI huntersEI[3] = { { 0.0 } };
	initializeTargetUSV(
		&target,
		targetPos,
		targetNormalVelocity,
		targetEscapingVelocity,
		targetAlertingRadius,
		targetSafetyRadius,
		targetMovableAngleRange,
		30
	);
	fors(
		huntersNum,
		initializeHunterUsv(
			&hunter[j],
			hunterPos[j],
			huntersVelocity,
			huntersMovableAngleRange,
			hunterHeadDirection[j]
		);
	);
	buildCycleList(hunter); //��ȫ��hunter�ṹ��LIST_ENTRY��
	fors(
		huntersNum,
		_SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(hunter[j], target); //��ȫ��hunter�ṹ��EI��
		_SURROUNDING_getRewardPointsForHunterByHuntersEI(hunter[j]);
		QAQ;
	);

	size_t cnt = 0x0;
	while (cnt < 250)
	{
		//���ڲ�ʳ���ٶ���Ŀ��δ����֮ǰ����Ŀ�꣬���Ժ�Χ100%��ʵ��.
		if (!isSurroundingSuccess(hunter, target)) //!isSurroundingSuccess(hunter, target)
		{
			//�������������ʳ�߶�����������ô��
			//1. (20%) һ��Ҫע��������ʳ�ߵ�����˳�򣡱���Ŀ����16 9����ô���3 2��1 8��λ�÷��ˣ��Ϳ��ܻ�������в�ʳ�߶��̶��������쳣����
			//2. (70%) �������˳����ȷ���ǲ�ʳ��׷����������Ȼ����ȫ��ͣ�Ͳ�����״�������Ե���J��w_idealAngleDiffer��Ȩ��ֵ��-16 9ʱ�����������ǰ�w_idealAngleDiffer��5��Ϊ25֮��������ˣ�
			//3. (10%) �ſ�һ������������Ŀǰ�����������ǱȽ��ϸ�ģ���maxDistanceTolerance��maxAngleTolerance��
			//ע1������ʱ��ֻ��Ҫ��while(1)��Ϊcnt < 200����.
			//ע2���������ĳ����ʳ����Ŀ��ľ���Խ��Խ��ͨ���ǽǶ�Ȩ��̫���˵��²�ʳ�߲����ֶ�����߽ǶȾ��ȶ������˾���Ȩ�أ�
			_SURROUNDING_changingHunterInfomationByRewardFunction(&hunter[0], target);
			_SURROUNDING_changingHunterInfomationByRewardFunction(&hunter[1], target);
			_SURROUNDING_changingHunterInfomationByRewardFunction(&hunter[2], target);
			//ע��printf�����������ӡ����ȷ���������safetyRadius + mostSafetyRadius��
			//ֻ���ڵ�����λ�ú󣬾�����������safetyRadius + mostSafetyRadius�������㣻
			//�����ӡ����ǰһ�ε�λ����Ϣ��������Ӿ��ϴ��󣬵��������ǶԵġ�
			printf("δ����. ��������: %zu -> [��ǰ������Ϣ] 0: %lf, 1: %lf, 2: %lf\n", cnt, calculate_P$P_distance(hunter[0]->pos, target->pos), calculate_P$P_distance(hunter[1]->pos, target->pos), calculate_P$P_distance(hunter[2]->pos, target->pos));
			//printfҲ���ܷ���movingNormalTargetRandomly֮�£�ԭ��ͬ�ϡ�
			movingNormalTargetRandomly(&target);
			makePythonFile(fp, hunter, target);
			QAQ;
			cnt++;
			continue;
		}
		else
		{
			printf("�Ѿ���������������: %zu -> [��ǰ������Ϣ] 0: %lf, 1: %lf, 2: %lf\n", cnt, calculate_P$P_distance(hunter[0]->pos, target->pos), calculate_P$P_distance(hunter[1]->pos, target->pos), calculate_P$P_distance(hunter[2]->pos, target->pos));
			printf("\n���һ����ʾ�ľ�����Ϣ���ܲ���������<���в�ʳ�ߵ�Ŀ��ľ���ȫ������mostSafetyRadius + 1 * RtN + 1 * Rh>\n\n���������ģ���Ϊ��ӡ����ϢʱĿ���Ѿ�������һ���ƶ���\n");
			makePythonFile(fp, hunter, target);
			QAQ;
			break;
		}
	}




	//���̽������ͷ�ʹ�õ��ڴ棬�˳�����
	fors(
		huntersNum,
		free(hunter[j]);
	hunter[j] = NULL;
		);
	free(target);
	target = NULL;
	fclose(fp);
	return 0;
}