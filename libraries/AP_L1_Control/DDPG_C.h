// QleaningUav.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <iostream>
#include <string>
#include <string.h>
#include <time.h>


using namespace std;

// I think Q matrix and R matrix must be same
// and it must be same both row and column
#define MATRIX_ROW 3
#define MATRIX_COLUMN 3
// state num must be equal to action num
//状态
#define STATE_NUM 3
//动作
#define ACTION_NUM 3
#define DES_STATE 2
#define MAX_EPISODE 1000
// this value using for Q(s,a)=R(s,a)+alpha * max{Q(s', a')}
// Q(s', a') is the all next step Q value
#define alpha 0.8

// define the most important Q-matrix and R-matrix
// R-matrix is given
//double R[100][100] = { {-1, -1, -1, -1, 0, -1},
//					 {-1, -1, -1, 0, -1, 100},
//					 {-1, -1, -1, 0, -1, -1},
//					 {-1, 0, 0, -1, 0, -1},
//					 {0, -1, -1, 0, -1, 100},
//					 {-1, 0, -1, -1, 0, 100} };
double R[100][100] = { {-1,-1,100},
                       {-1,100,0},
                       {-1,-1,100}};
//double Q[100][100];

int inference_best_action(int now_state, double Q[100][100]) {
	// get the max value of Q corresponding action when state is nw_state
	double temp_max_q = 0;
	int best_action = 0;
	for (int i = 0; i < ACTION_NUM; ++i) {
		if (Q[now_state][i] > temp_max_q) {
			temp_max_q = Q[now_state][i];
			best_action = i;
		}
	}
	return best_action;
}

/*
int main() {

	cout << "Q matrix:" << endl;
	print_matrix(Q, 3, 3);
	cout << "R matrix:" << endl;
	print_matrix(R, 3, 3);

	run_training(0);//训练
	cout << "Q convergence matrix:" << endl;
	print_matrix(Q, 3, 3);

	int position;
	while (1) {
		cout << "please input robot locate room: " << endl;
		cin >> position;
		cout << position << "->";
		while (1) {
			int best_action = inference_best_action(position, Q);
			cout << best_action << "->";
			if (best_action == DES_STATE) {
				cout << "out" << endl;
				break;
			}
			else {
				position = best_action;
			}
		}
	}

	return 0;
}
*/

