#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

// node sturch for the branch and bound algorithm
struct node {

	vector<int> state;
	node* parent;
	int depth;
	vector<bool> marker;
	vector<vector<int>> applicable;

};

// read input values from the standard input
void readInput(int& N, vector<int>& rowSum, vector<int>& colSum) {

	cout << "Enter size of the matrix (NxN):" << endl;
	cin >> N;

	int val;

	cout << "Enter rowSum values:" << endl;
	rowSum.clear();
	for (int i = 0; i < N; i++) {
		cin >> val;
		rowSum.push_back(val);
	}

	cout << "Enter colSum values:" << endl;
	colSum.clear();
	for (int i = 0; i < N; i++) {
		cin >> val;
		colSum.push_back(val);
	}

}

// print a vector of ints
void printVector(const vector<int>& vec) {

	for (int i = 0; i < vec.size(); i++) {
		cout << vec[i] << " ";
	}

	cout << endl;

}

// print a vector of bools
void printVector(const vector<bool>& vec) {

	for (int i = 0; i < vec.size(); i++) {
		cout << vec[i] << " ";
	}

	cout << endl;

}

// check if the given node ha a goal state
bool isGoal(node* n, const vector<int>& colSum) {

	// allocate array for storing sum of cols, and initialize it with zeros
	int* sum = new int[colSum.size()];

	for (int i = 0; i < colSum.size(); i++) {
		sum[i] = 0;
	}

	// pointer for iterating over nodes
	node* tmp = n;

	while (tmp->parent != nullptr) {

		for (int i = 0; i < tmp->state.size(); i++) {
			sum[i] += tmp->state[i];
		}

		tmp = tmp->parent;

	}

	// check if the actual sum is equal with the needed colsum
	bool result = true;

	for (int i = 0; i < colSum.size(); i++) {
		if (colSum[i] != sum[i]) {
			result = false;
			break;
		}
	}

	// free memory
	delete sum;

	return result;

}

// track 1-to-0 transitions
vector<bool> oneToZeroTransition(const vector<int>& from, const vector<int>& to) {

	vector<bool> result;

	for (int i = 0; i < from.size(); i++) {
		result.push_back(from[i]==1 && to[i]==0);
	}

	return result;

}

// helper function for keeping v-convexity
bool isOperatorCompatibleWithMarker(const vector<int>& op, const vector<bool>& marker) {

	for (int i = 0; i < op.size(); i++) {

		if (op[i] == 1 && marker[i] == true) {

			return false;

		}

	}

	return true;

}

// perform logical or operation between two boolean vectors
vector<bool> boolVectorLogicalOr(const vector<bool>& a, const vector<bool>& b) {

	vector<bool> result;

	for (int i = 0; i < a.size(); i++) {
		result.push_back(a[i] || b[i]);
	}

	return result;

}

// construct and print out the solution in a correct order to the standard output
void printSolution(node* goal) {

	// get number of rows and columns
	int N = goal->state.size();

	// pointer for iterating over nodes
	node* tmp = goal;

	// allocate new 2d array
	int** matrix = new int*[N];

	// fill values of the matrix
	int i = N - 1;
	while (tmp->parent != nullptr) {

		matrix[i] = new int[N];

		for (int j = 0; j < N; j++) {
			matrix[i][j] = tmp->state[j];
		}

		i--;

		tmp = tmp->parent;

	}

	// print solution
	cout << "------------ SOLUTION ------------" << endl;
	for (i = 0; i < N; i++) {

		for (int j = 0; j < N; j++) {
			cout << matrix[i][j] << " ";
		}

		cout << endl;

	}
	cout << "----------------------------------" << endl;
	
	// free memory
	for (int i = 0; i < N; i++) {
		delete matrix[i];
	}
	delete[] matrix;

}

// binary tomography reconstrucion branch and bound algorithm implementation
// the algorithm only accepts hv-convex results
void branchAndBound(const int& N, const vector<int>& rowSum, const vector<int>& colSum) {
	
	// init the root node
	node* actual = new node;
	actual->depth = 0;
	actual->parent = nullptr;

	actual->applicable.clear();
	for (int i = 0; i < N - rowSum[0] + 1; i++) {

		// init vector with size N filled with zeros
		vector<int> op(N, 0);

		// put in ones
		for (int j = 0; j < rowSum[0]; j++) {
			op[i + j] = 1;
		}

		// add operator to applicable
		actual->applicable.push_back(op);

	}

	// main loop of the algorithm
	while (true) {

		if (actual == nullptr) {
			break;
		}

		if (isGoal(actual, colSum)) {

			// print the state
			printSolution(actual);

			// dont stop, search for other solutions too
			// also free memory
			node* tmp = actual;
			actual = actual->parent;
			delete tmp;

		}

		if (actual->depth == N) {

			// step back
			// also free memory
			node* tmp = actual;
			actual = actual->parent;
			delete tmp;

		}

		if (!actual->applicable.empty()) {

			// choose operator
			vector<int> op(actual->applicable[0]);

			// remove chosen operator
			actual->applicable.erase(actual->applicable.begin() + 0);

			// check operator, to keep v-convexity
			if (actual->depth >= 1) {

				if (!isOperatorCompatibleWithMarker(op, actual->marker)) {

					continue;

				}

			}

			// init new node
			node* newNode = new node;
			newNode->state = op;
			newNode->parent = actual;
			newNode->depth = actual->depth + 1;
			if (actual->depth >= 1) {
				newNode->marker = boolVectorLogicalOr(actual->marker, oneToZeroTransition(actual->state, op));
			}
			else {
				newNode->marker = vector<bool>(N, false);
			}

			// if we are not on a leaf node, we init the applicable vector for the new node
			if (actual->depth + 1 < N) {

				// init applicable
				newNode->applicable.clear();
				for (int i = 0; i < N - rowSum[actual->depth + 1] + 1; i++) {

					// init vector with size N filled with zeros
					vector<int> op(N, 0);

					// put in ones
					for (int j = 0; j < rowSum[actual->depth + 1]; j++) {
						op[i + j] = 1;
					}

					// add operator to applicable
					newNode->applicable.push_back(op);

				}

			}

			// change pointer
			actual = newNode;

		}
		else {

			// step back
			// also free memory
			node* tmp = actual;
			actual = actual->parent;
			delete tmp;

		}

	}

	// free memory
	delete actual;

}

int main() {

	int N;
	vector<int> rowSum;
	vector<int> colSum;

	readInput(N, rowSum, colSum);

	branchAndBound(N, rowSum, colSum);

	return 0;

}