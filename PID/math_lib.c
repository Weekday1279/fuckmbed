/*
 * @Author: CloudSir
 * @Date: 2021-08-30 15:41:25
 * @LastEditTime: 2021-08-30 15:41:40
 * @LastEditors: CloudSir
 * @Description: 
 * https://github.com/cloudsir
 */

// ����ӳ�亯��
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// �������ַ�Χ
float Clamp(float x, float min, float max)
{
    if (x > max)
        return max;
    if (x < min)
        return min;
    return x;
}
