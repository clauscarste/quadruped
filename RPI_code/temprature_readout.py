import max6675
import math
def is_temp_in_limit(limit_temp):
    max6675.set_pin()
    R = max6675.read_temp()
    R = R if R else 10
    celsius = ((1 / (-0.0006409971753509394 + 0.0005829519842237316 * math.log(R) + -0.0000011723458332861173 * math.log(R) * math.log(R) * math.log(R))) - 273.15)
    out = True if celsius<limit_temp else False
    if celsius>limit_temp:
        print("motor_has exeeded the temprature limit of",limit_temp,"°C, currently at", celsius,"°C")
    # if thermistor loses conection R goes to 0, to avoid overheating and show tat something is wrong and also avoid a devide by zero error value 10(equates to more than 1000°) is replaced to trip the limit.
    return(out)




