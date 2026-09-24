#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <sstream>
#include <string>
#include <io.h>
#include <vector>
#include <iomanip>
#include<algorithm>
#include<cmath>
#define MAXI 20000
using namespace std;
vector<string> split(string line, const string& separator) {

	vector<string> list;
	size_t pos;
	string token;
	line += separator;
	while ((pos = line.find(separator)) != string::npos) {
		token = line.substr(0, pos);
		if (!token.empty()) list.push_back(token);
		line.erase(0, pos + 1);
	}
	return list;
}

const char * best_result = "best_result.txt";
const char * best_time = "best_time.txt";
const char * average_value = "average_value.txt";
const char * average_time = "average_time.txt";
int res;
double  a[MAXI][MAXI] = { 0.0 };
struct node {
	char value[20] = "";
	node *next;
};
typedef node *list;
struct node1 {
	double result;
	double time1;
	node1 * next;
};
typedef node1 *list1;
double result[1000];
double arg_result[1000] = { 0 };
double temp_time[1000], arg_time[1000] = { 0.0 };
int Simure[MAXI] = { 0 };
double logtime[MAXI] = { 0.0 };
//list lamdalist, total_list;
list1 bestlist, arglist;
node1 * kk, *kk2;

void GetLog(string in_name, string log_name, string best_log)
{
	int counter = 0;
	double temp_best_value, tempbestTime, temp_average_value, temp_average_time, temp_value;
	int index = -1;
	ifstream fin(in_name);
	if (!fin)
	{
		cerr << "open file error" << endl;
		//exit(-1);
	}
	string str;
	temp_best_value = 999999999;
	while (getline(fin, str))
	{
		if (str[0] != 'C' || str.size() == 0) continue;
		vector<string> tokens = split(str, " ");
		//cout << tokens[0] << " "<<tokens[1]<< endl;
		if (tokens[0] == "Cost") {
			counter++;
			temp_value = stod(tokens[1]);
			if (temp_best_value - 1e-6 > temp_value) {
				temp_best_value = temp_value;
				index = counter;
			}
		}
	}

	ifstream fin2(log_name);
	counter = 0;
	int min = (index - 1) * 20000;
	temp_average_value = 0;
	int max = min + 20000;
	while (getline(fin2, str))
	{
		counter++;
		/*********  Multistart **********/
		if (counter <= min) continue;
		else if (counter > max) break;
		else { Simure[counter - min - 1] = stoi(str); temp_average_value += Simure[counter - min - 1]; }
	}
	cout << index << " " << temp_average_value * 1.0 / 20000 << endl;
	ofstream fout(best_log);
	for (int i = 0; i < 20000; i++) {
		fout << Simure[i] << endl;
	}
	fout.close();
	// statistic maximize
	//temp_best_value = 0;
	//for (int i = 0; i < counter; i++) {
	//	if (temp_best_value < result[i]) {
	//		temp_best_value = result[i];
	//		tempbestTime = temp_time[i];
	//	}
	//	//cout << result[i] << " ";
	//	temp_average_value += result[i];
	//	temp_average_time += temp_time[i];
	//}

	// statistic minimize 
	temp_average_value = temp_average_value * 1.0 / 20000;
	node1 * temp_node = new node1;
	temp_node->next = NULL;
	temp_node->result = temp_best_value;
	temp_node->time1 = tempbestTime;
	kk->next = temp_node;
	kk = temp_node;
	node1 * temp_node2 = new node1;
	temp_node2->next = NULL;
	temp_node2->result = temp_average_value;
	temp_node2->time1 = temp_average_time;
	kk2->next = temp_node2;
	kk2 = temp_node2;
}
void GetLineAndPrint(string in_name)
{
	int counter = 0;
	double temp_best_value, tempbestTime, temp_average_value, temp_average_time, temp_value;
	ifstream fin(in_name);
	if (!fin)
	{
		cerr << "open file error" << endl;
		//exit(-1);
	}
	string str;

	while (getline(fin, str))
	{
		/********* Sim statistic**********/
		//if (str[0] != 'S'  || str.size() == 0) continue;
		//vector<string> tokens = split(str, " ");
		////cout << tokens[0] << " "<<tokens[1]<< endl;
		//if (tokens[0] == "Sim") { result[counter] = stoi(tokens[1]);counter++;}

		//else if (tokens[0] == "Time") { temp_time[counter] = stod(tokens[1]); counter++; }
		/*********  Multistart **********/
		result[counter] = stod(str);
		counter++;

		/************ result of QUBO *************/
		////if (str[0] != 'P' || str.size() == 0) continue;
		//vector<string> tokens = split(str, ":");
		////cout << tokens[0] << " "<<tokens[1]<< endl;
		///*if (tokens[0] == "O") { result[counter] = stod(tokens[1]);  }

		//else if (tokens[0] == "PT") { temp_time[counter] = stod(tokens[1]); counter++; }*/
		//if (tokens[0] == "O") { result[counter] = stod(tokens[1]); }

		//else if (tokens[0] == "T") { temp_time[counter] = stod(tokens[1]); counter++; }
	}

	// statistic maximize
	//temp_best_value = 0;
	//for (int i = 0; i < counter; i++) {
	//	if (temp_best_value < result[i]) {
	//		temp_best_value = result[i];
	//		tempbestTime = temp_time[i];
	//	}
	//	//cout << result[i] << " ";
	//	temp_average_value += result[i];
	//	temp_average_time += temp_time[i];
	//}

	// statistic minimize 

	temp_best_value = 999999999;
	for (int i = 0; i < counter; i++) {
		if (temp_best_value > result[i]) {
			temp_best_value = result[i];
			tempbestTime = temp_time[i];
		}
		//cout << result[i] << " ";
		temp_average_value += result[i];
		temp_average_time += temp_time[i];
	}
	cout << endl;
	cout << temp_best_value;
	node1 * temp_node = new node1;
	temp_node->next = NULL;
	temp_node->result = temp_best_value;
	temp_node->time1 = tempbestTime;
	kk->next = temp_node;
	kk = temp_node;
	node1 * temp_node2 = new node1;
	temp_node2->next = NULL;
	temp_node2->result = temp_average_value / counter;
	temp_node2->time1 = temp_average_time / counter;
	kk2->next = temp_node2;
	kk2 = temp_node2;

}
double average(int *x, int len) {
	double sum = 0;
	for (int i = 0; i < len; i++)sum += x[i];
	return (double)(sum *1.0 / len);
}
double SampleStandardDeviation(int *x, int len, double avg) {
	double sum = 0;
	for (int i = 0; i < len; i++) {
		sum += pow(x[i] - avg, 2);
	}
	return sqrt(sum / (len - 1));
}
void GetLineAndPrint3(string in_name)
{

	int counter = 0;
	double temp_best_value, tempbestTime, temp_average_value, temp_average_time, temp_value;
	double worse_value = 0;
	ifstream fin(in_name);
	if (!fin)
	{
		cerr << "open file error" << endl;
		//exit(-1);
	}
	string str;

	while (getline(fin, str))
	{
		//line int   :BestLog
		Simure[counter] = stod(str);
		counter++;


		//line: int, double :Prelog,Postlog
		//std::istringstream iss(str);
		//int firstValue;
		//double secondValue;
		//char comma; // 用于处理逗号
		//if (iss >> firstValue >> comma >> secondValue && comma == ',') {
		//	Simure[counter] = firstValue;
		//	logtime[counter] = secondValue;
		//	counter++;
		//}
		//else {
		//	std::cerr << "解析行失败: " << str << std::endl;
		//}

	}
	sort(Simure, Simure + counter);
	temp_best_value = 999999999;
	temp_average_time = 0;
	for (int i = 0; i < counter; i++) {
		if (temp_best_value > Simure[i]) {
			temp_best_value = Simure[i];
			tempbestTime = temp_time[i];
		}
		else if (worse_value < Simure[i]) {
			worse_value = Simure[i];

		}
		//cout << result[i] << " ";
		temp_average_value += Simure[i];
		temp_average_time += logtime[i];
		//temp_average_time += temp_time[i];
	}

	cout << "  " << temp_average_value / counter << endl;
	node1 * temp_node = new node1;
	temp_node->next = NULL;
	temp_node->result = Simure[19800];//Simure[19800]
	temp_node->time1 = worse_value;
	kk->next = temp_node;
	kk = temp_node;
	node1 * temp_node2 = new node1;
	temp_node2->next = NULL;
	temp_node2->result = temp_average_value / counter;
	temp_node2->time1 = temp_average_time / counter;// Simure[16000]
	kk2->next = temp_node2;
	kk2 = temp_node2;

}

void StandardDeviation(string in_name)
{

	int counter = 0;
	double temp_best_value, tempbestTime, temp_average_value, temp_average_time, temp_value;
	double worse_value = 0;
	ifstream fin(in_name);
	if (!fin)
	{
		cerr << "open file error" << endl;
		//exit(-1);
	}
	string str;

	while (getline(fin, str))
	{
		Simure[counter] = stod(str);
		counter++;
	}
	temp_average_value = average(Simure, 20000);
	temp_best_value = SampleStandardDeviation(Simure, 20000, temp_average_value);
	tempbestTime = temp_best_value / temp_average_value * 100;
	cout << endl;
	cout << temp_best_value;
	node1 * temp_node = new node1;
	temp_node->next = NULL;
	temp_node->result = temp_best_value;
	temp_node->time1 = tempbestTime;
	kk->next = temp_node;
	kk = temp_node;
	node1 * temp_node2 = new node1;
	temp_node2->next = NULL;
	temp_node2->result = temp_average_value;
	temp_node2->time1 = Simure[16000];
	kk2->next = temp_node2;
	kk2 = temp_node2;

}
int main()
{
	int num_instance;
	int fixed = 6;
	string in_path = "";
	cout << "input num of res:" << endl;
	cin >> res;
	cout << "input num of instances:" << endl;
	cin >> num_instance;
	cout << "input file of results:" << endl;
	cin >> in_path;
	cout << "input the accuracy:" << endl;
	cin >> fixed;
	string post = "";
	cout << "input the Variant:" << endl;
	cin >> post;
	bestlist = new node1;
	bestlist->next = NULL;
	kk = bestlist;
	arglist = new node1;
	arglist->next = NULL;
	kk2 = arglist;
	struct _finddata_t fileinfo;
	//string in_path = "";
	string in_name;
	string log_name;
	string best_log_name;
	string file_name[1000];
	string instancesflie = "instance.txt";
	ifstream cinn(instancesflie);
	for (int i = 0; i < num_instance; i++) {
		cinn >> file_name[i];
	}
	cinn.close();
	for (int i = 0; i < num_instance; i++) {

		in_name = in_path + file_name[i] + "_sol.txt";
		log_name = in_path + file_name[i] + "_sollog.txt";
		best_log_name= in_path+ file_name[i] + "_Bestlog.txt";
		cout << in_name << endl;
		GetLog(in_name,log_name,best_log_name);

		//****multi start time ***//
		//in_name = in_path + file_name[i] + "_sol"+post+".txt";
		//cout << in_name << endl;
		//GetLineAndPrint(in_name);

		//in_name = in_path + file_name[i] + "_Bestlog.txt";// _Bestlog
		//cout << in_name;
		//GetLineAndPrint3(in_name);
		//StandardDeviation(in_name);
	}
	cout << "end" << endl;
	ofstream out1(best_result);
	ofstream out2(best_time);
	ofstream out3(average_value);
	ofstream out4(average_time);
	node1 * h1, *h2;
	out1 << setiosflags(ios::fixed) << setprecision(fixed);
	out2 << setiosflags(ios::fixed) << setprecision(fixed);
	out3 << setiosflags(ios::fixed) << setprecision(fixed);
	out4 << setiosflags(ios::fixed) << setprecision(fixed);
	h1 = bestlist->next;
	while (h1 != NULL) {
		out1 << h1->result << endl;
		out2 << h1->time1 << endl;
		h1 = h1->next;
	}

	h2 = arglist->next;
	while (h2 != NULL) {
		out3 << h2->result << endl;
		out4 << h2->time1 << endl;
		h2 = h2->next;
	}
	out1.close();
	out2.close();
	out3.close();
	out4.close();
	cout << "end2" << endl;
	/*delete*/
	node1 *del;
	h1 = bestlist;
	h2 = arglist;
	while (h1 != NULL) {
		del = h1;
		h1 = h1->next;
		delete del;
	}
	while (h2 != NULL) {
		del = h2;
		h2 = h2->next;
		delete del;
	}
	system("pause");
	return 0;
}
