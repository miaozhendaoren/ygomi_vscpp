#pragma once

#include "typeDefine.h"

enum ActionType_e{
    CREATE,
    UPDATE,
    DELETE
};

/*
*For Date_t and Time_t,
*it is recommended that get initial from standed C library time.h,
*see references about:
*    time_t t = time(NULL);
*    struct tm *local = localtime(&t);
*/

struct Date_t {
    int tm_mday; // 1 ~ 31
    int tm_mon;  // 1 ~ 12
    int tm_year; // current year,like 2015, 2016, ...

    Date_t() {
        tm_mday = 0;
        tm_mon = 0;
        tm_year = 0;
    }
};

struct Time_t{
    int tm_sec;
    int tm_min;
    int tm_hour;

    Time_t() {
        tm_sec = 0;
        tm_min = 0;
        tm_hour = 0;
    }
};


namespace ns_database {

class Difference
{
public:
    Difference(void);
    Difference(int32 differenceId, int32 vehicleId, Date_t date, Time_t time);
    ~Difference(void);

public:
    int32 getId() const;
    int32 getDifferenceId() const;
    int32 getVehicleId() const;
    Date_t getDate() const;
    Time_t getTime() const;

    void setId(int32);
    void setDifferenceId(int32);
    void setVehicleId(int32);
    void setDate(Date_t);
    void setTime(Time_t);
private:
    int32 mId;
    int32 mDifferenceId;
    Date_t mDate;
    Time_t mTime;
    int32 mVehicleId;
};


}
