/* This file is there to support additional attributes in
 sysfs - changestate and setvoltage 
*/
#ifndef _DA9052_REGULATOR_H
#define _DA9052_REGULATOR_H

int da9052_ldo_buck_enable(struct regulator_dev *rdev);
int da9052_ldo_buck_disable(struct regulator_dev *rdev);


int da9052_ldo_buck_set_voltage(struct regulator_dev *rdev,
                                        int min_uV, int max_uV);
int da9052_ldo_buck_get_voltage(struct regulator_dev *rdev);

#endif
