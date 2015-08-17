#include <iostream>
#include <stdio.h>
#include "Sign.h"
#include "DataBaseManager.h"
#include "dbtools.h"
#include "log.h"

using namespace ns_database;
using namespace ns_databasemanager;

void test01(){
    double d = sin(PI/2); // 90
    printf("sin(Pi/2) = %lf\n", d);
    d = sin(PI/6); //30
    printf("sin(Pi/6) = %lf\n", d);

    d = sin(MEASURE_DD2RADIAN(90)); //90
    printf("sin(90) = %lf\n", d);

    d = sin(MEASURE_DD2RADIAN(30)); //30
    printf("sin(30) = %lf\n", d);

    d = COEFF_DD2METER_ON_LAT(32);
    printf("The distance of per longitude on NanJing (North latitude 32 degree) is %lf meters\n", d);

    d = COEFF_DD2METER_ON_LAT(0);
    printf("The distance of per longitude on Equator degree is %lf meters\n", d);
}


void test02() {
    int a = 1;
    int b = 0;
    char* s =  "Hello world!";

    WARN_IF(a> b);
    ERR_IF(a > b);

    WARN_IF(b > a);
    ERR_IF(b > a);

    LOGI("a: %d, b: %d, c: %s\n", a, b, s);
    LOGD("a: %d, b: %d, c: %s\n", a, b, s);
    LOGW("a: %d, b: %d, c: %s\n", a, b, s);
    LOGE("a: %d, b: %d, c: %s\n", a, b, s);

}

void test03(){
	PointInCCS p1(3,7,0);
	PointInCCS p2(-1,-1,0);
	PointInCCS p3(8,17,0);
	PointInCCS p4(2,3,0);
	PointInCCS p5(20,3,0);
	PointInCCS pM(0,1,0);
	PointInCCS pN(5,11,0);

	printf("res p1: %d\n",DataBaseMathBase::pointInLine(p1,pM,pN));
	printf("res p2: %d\n",DataBaseMathBase::pointInLine(p2,pM,pN));
	printf("res p3: %d\n",DataBaseMathBase::pointInLine(p3,pM,pN));
	printf("res p4: %d\n",DataBaseMathBase::pointInLine(p4,pM,pN));
	printf("res p5: %d\n",DataBaseMathBase::pointInLine(p5,pM,pN));
}

int main (int argc, char* argv[]) {
    /*Sign sign;
    insertSignToMasterDB(sign);

    ns_dbmaster::DBTools* pDBtools = ns_dbmaster::DBTools::getInstance();

    test01();
    test02();*/
	test03();
    std::cin.get();
    return 0;
}