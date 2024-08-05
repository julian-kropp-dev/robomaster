#include "header/Transformations.h"

float Transformations::radians(float direction) {
	return direction / 180.0f * M_PI;
}

float Transformations::degrees(float radians) {
	return radians / M_PI * 180.0f;
}

Vector Transformations::getVectorOfDirection(float direction) {
	float r = radians(direction);
	return Vector(cos(r), sin(r), 0.0f);
}

float Transformations::dotProduct(Vector v1, Vector v2) {
	double result = 0.0f;
	result += (v1.x * v2.x);
	result += (v1.y * v2.y);
	result += (v1.z * v2.z);

	result = std::min(std::max(result, -1.), 1.);
	return result;
}

Vector Transformations::crossProduct(Vector v1, Vector v2) {
	return Vector((v1.y * v2.z) - (v1.z * v2.y),
		(v1.z * v2.x) - (v1.x * v2.z),
		(v1.x * v2.y) - (v1.y * v2.x));
}

float Transformations::SignedAngleBetweenVectors(Vector v1, Vector v2, Vector vn) {
	float angle = degrees(acos(dotProduct(v1.normalize(), v2.normalize())));
	Vector cross = crossProduct(v1, v2);
	if (dotProduct(vn, cross) < 0) { // Or > 0
		angle = -angle;
	}
	return angle;
}

float Transformations::SignedAngleBetweenVectorAndDirection(Vector v2, float direction) {
	Vector v = getVectorOfDirection(direction);
	Vector vn = Vector(0.0f, 0.0f, 1.0f);

	return SignedAngleBetweenVectors(v, v2, vn);
}

float degreesDebug(float radians) {
	std::cout << "radians: " << radians << std::endl;
	return radians / M_PI * 180.0f;
}

Vector crossProductDebug(Vector v1, Vector v2) {
	return Vector((v1.y * v2.z) - (v1.z * v2.y),
		(v1.z * v2.x) - (v1.x * v2.z),
		(v1.x * v2.y) - (v1.y * v2.x));
}

float dotProductDebug(Vector v1, Vector v2) {
	std::cout << "dotProduct  X: " << v1.x << " Y: " << v1.y << " Z: " << v1.z << " ";
	std::cout << "dotProduct  X: " << v2.x << " Y: " << v2.y << " Z: " << v2.z << " ";
	double result = 0.0f;
	result += (v1.x * v2.x);
	result += (v1.y * v2.y);
	result += (v1.z * v2.z);

	std::cout << "dotProduct: " << result << " ";
	result = std::min(std::max(result, -1.), 1.);
	return result;
}

float SignedAngleBetweenVectorsDebug(Vector v1, Vector v2, Vector vn) {
	float angle = degreesDebug(acos(dotProductDebug(v1.normalize(), v2.normalize())));
	Vector cross = crossProductDebug(v1, v2);
	std::cout << angle << std::endl;
	std::cout << "Cross  X: " << cross.x << " Y: " << cross.y << " Z: " << cross.z << std::endl;
	if (dotProductDebug(vn, cross) < 0) { // Or > 0
		angle = -angle;
	}
	std::cout << angle << std::endl;
	return angle;
}

float Transformations::SignedAngleBetweenVectorAndDirection(Vector v2, float direction, bool debug) {
	Vector v = getVectorOfDirection(direction);
	if (debug) {
		std::cout << "  X: " << v.x << " Y: " << v.y << " Z: " << v.z << std::endl;
		std::cout << "  X: " << v2.x << " Y: " << v2.y << " Z: " << v2.z << std::endl;
	}
	Vector vn = Vector(0.0f, 0.0f, 1.0f);

	return SignedAngleBetweenVectorsDebug(v, v2, vn);
}


/*float positionsberechnungX(float directionz, float xz, float yz, float forward)
{
	xz = xz + forward * cos(radians(directionz));
	return xz;
}

float positionsberechnungY(float directionz, float xz, float yz, float forward)
{
	yz = yz + forward * sin(radians(directionz));

	return yz;
}

float distance(float ax, float ay, float az, float zx, float zy, float zz)
{
	float distance = 0;

	float zw1 = 0;
	float ajez = ay - zy;
	ajez = ajez * ajez;
	zw1 = ax - zx;
	zw1 = zw1 * zw1;
	float zw2 = az - zz;
	zw2 = zw2 * zw2;
	ajez = ajez + zw1 + zw2;
	zw1 = 0;
	zw2 = 0;
	distance = sqrt(ajez);

	return distance;
}

float drehberechnungM(float aktpoint1, float aktpoint2, float aktpoint3, float posx, float posy, float direction)
{
	float dreh = 0;
	float aktvektor[] = new float[3];
	float sehvektor[] = new float[3];
	float l[] = new float[2];
	float pos[] = new float[3];
	float aktpoint[] = new float[3];

	aktpoint[0] = aktpoint1;
	aktpoint[1] = aktpoint2;
	aktpoint[2] = aktpoint3;

	pos[0] = posx;
	pos[1] = posy;

	sehvektor[0] = cos(radians(direction));
	sehvektor[1] = sin(radians(direction));

	aktvektor[0] = (aktpoint[0] - pos[0]);
	aktvektor[1] = (aktpoint[1] - pos[1]);
	aktvektor[2] = aktpoint[2] - pos[2];

	aktvektor[0] = aktvektor[0] * sehvektor[0];
	aktvektor[1] = aktvektor[1] * sehvektor[1];

	aktvektor[2] = aktvektor[0] + aktvektor[1];

	aktvektor[0] = (aktpoint[0] - pos[0]);
	aktvektor[1] = (aktpoint[1] - pos[1]);

	l[0] = sqrt(aktvektor[0] * aktvektor[0] + aktvektor[1] * aktvektor[1]);
	l[1] = sqrt(sehvektor[0] * sehvektor[0] + sehvektor[1] * sehvektor[1]);

	l[1] = l[1] * l[0];

	aktvektor[2] = aktvektor[2] / l[1];

	dreh = degrees(acos(aktvektor[2]));

	//println("Dis0: "+dreh);
	//println("Dis1: "+distance(positionsberechnungX(dreh+direction,pos[0],pos[1],l[0]),positionsberechnungY(dreh,pos[0],pos[1],l[0]),0,aktpoint1,aktpoint2,0));
	//println("Dis2: "+distance(positionsberechnungX((360-dreh)+direction,pos[0],pos[1],l[0]),positionsberechnungY(dreh,pos[0],pos[1],l[0]),0,aktpoint1,aktpoint2,0));



	if (distance(positionsberechnungX(dreh + direction, pos[0], pos[1], l[0]), positionsberechnungY(dreh + direction, pos[0], pos[1], l[0]), 0, aktpoint1, aktpoint2, 0) < distance(positionsberechnungX(direction - dreh, pos[0], pos[1], l[0]), positionsberechnungY(direction - dreh, pos[0], pos[1], l[0]), 0, aktpoint1, aktpoint2, 0))
	{
		dreh = int(degrees(acos(aktvektor[2])));
	}
	else
	{
		dreh = -int(degrees(acos(aktvektor[2])));
	}

	return dreh;
}


float positionsberechnungzX(float ajdir, float xz, float yz, float forward)
{
	xz = xz + forward * sin(radians(ajdir));
	return xz;
}

float positionsberechnungzY(float ajdir, float xz, float yz, float forward)
{
	yz = yz + forward * cos(radians(ajdir));
	return yz;
}

float distance2(int ax, int ay, int az, int zx, int zy, int zz)
{
	float distance = 0;

	float zw1 = 0;
	float ajez = ay - zy;
	ajez = ajez * ajez;
	zw1 = ax - zx;
	zw1 = zw1 * zw1;
	float zw2 = az - zz;
	zw2 = zw2 * zw2;
	ajez = ajez + zw1 + zw2;
	zw1 = 0;
	zw2 = 0;
	distance = sqrt(ajez);

	return distance;
}

float distanced(int ax, int ay, int az, int zx, int zy, int zz)
{
	float distance = 0;

	if (ax > zx)
	{
		distance = ax - zx;
	}
	else
	{
		distance = zx - ax;
	}

	return distance;
}*/
