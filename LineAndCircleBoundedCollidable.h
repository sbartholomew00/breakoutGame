#pragma once
#include <set>
#include <vector>

struct float2 {
	float x;
	float y;
};

bool operator==(const float2& a, const float2& b);

bool operator!=(const float2& a, const float2& b);

float2 operator*(const float2& x, float a);

float2 operator*(float a, const float2& x);

float2 operator/(const float2& x, float a);

float2 operator+(const float2& a, const float2& b);

float2 operator-(const float2& a, const float2& b);

float2& operator+=(float2& a, const float2& b);

float2& operator-=(float2& a, const float2& b);

float2 operator-(const float2& a);

float dotProduct(const float2& a, const float2& b);

struct Matrix2x2 {
	float xx;
	float xy;
	float yx;
	float yy;
};

float2 operator*(const Matrix2x2& a, const float2& x);

Matrix2x2 operator+(const Matrix2x2& a, const Matrix2x2& b);

struct Line;
struct Circle;

class LineAndCircleBoundedCollidable
{
	struct comparisonFunction {
		bool operator()(const LineAndCircleBoundedCollidable* const a, const LineAndCircleBoundedCollidable* const b) const;
	};

	static std::set<LineAndCircleBoundedCollidable*, comparisonFunction> collidables;
	float2 location;
	float2 velocity;
	float timeOfCollision;
	float timeAhead;
	LineAndCircleBoundedCollidable* nextPossibleCollision;
	std::vector<Line> lines;
	std::vector<Circle> circles;
	float2 forceVec;

	LineAndCircleBoundedCollidable& operator=(const LineAndCircleBoundedCollidable&) = delete;
	LineAndCircleBoundedCollidable(const LineAndCircleBoundedCollidable&) = delete;

	void checkForNextCollision();
	void updateListPosition(float newTimeOfCollision);
	virtual void onCollision() {}
	virtual float getCorFactorPerp() { return 1.0f; }
	virtual float getCorFactorTang() { return 1.0f; }
	virtual const Matrix2x2 getInverseMassMatrix() = 0;
public:
	static void doTickOfCollisions();
	LineAndCircleBoundedCollidable(const float2& initLocation, const float2& initVelocity);
	~LineAndCircleBoundedCollidable();
	LineAndCircleBoundedCollidable(LineAndCircleBoundedCollidable&&) noexcept;
	LineAndCircleBoundedCollidable& operator=(LineAndCircleBoundedCollidable&&) noexcept;
	void changeTrajectory(const float2& newLocation, const float2& newVelocity);
	void changeVelocity(const float2& newVelocity);
	void addLine(const float2& p1, const float2& p2); // Lines should be added with p2 clockwise from p1 for collision with stuff outside
	void addCircle(const float2& centre, float radius);
	float2 getLocation() { return location; }
	float2 getVelocity() { return velocity; }
};

