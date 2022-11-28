#include "LineAndCircleBoundedCollidable.h"
#include <string>

std::set<LineAndCircleBoundedCollidable*, LineAndCircleBoundedCollidable::comparisonFunction> LineAndCircleBoundedCollidable::collidables{};

bool operator==(const float2& a, const float2& b) {
    return (a.x == b.x) && (a.y == b.y);
}

bool operator!=(const float2& a, const float2& b) {
    return !((a.x == b.x) && (a.y == b.y));
}

float2 operator*(const float2& x, float a) {
    return { a * x.x,a * x.y };
}

float2 operator*(float a, const float2& x) {
    return { a * x.x,a * x.y };
}

float2 operator/(const float2& x, float a) {
    return { x.x / a,x.y / a };
}

float2 operator+(const float2& a, const float2& b) {
    return { a.x + b.x,a.y + b.y };
}

float2 operator-(const float2& a, const float2& b) {
    return { a.x - b.x,a.y - b.y };
}

float2& operator+=(float2& a, const float2& b) {
    a.x = a.x + b.x;
    a.y = a.y + b.y;
    return a;
}

float2& operator-=(float2& a, const float2& b) {
    a.x = a.x - b.x;
    a.y = a.y - b.y;
    return a;
}

float2 operator-(const float2& a) {
    return { -a.x,-a.y };
}

float dotProduct(const float2& a, const float2& b) {
    return a.x * b.x + a.y * b.y;
}

float2 operator*(const Matrix2x2& a, const float2& x) {
    return { a.xx * x.x + a.xy * x.y,a.yx * x.x + a.yy * x.y };
}

Matrix2x2 operator+(const Matrix2x2& a, const Matrix2x2& b) {
    return { a.xx + b.xx,a.xy + b.xy,a.yx + b.yx,a.yy + b.yy };
}

struct Line {
    float2 p1;
    float2 p2;
};
Line operator+(const Line& line, const float2& offset) {
    return { line.p1 + offset,line.p2 + offset };
}
Line operator-(const Line& line, const float2& offset) {
    return { line.p1 - offset,line.p2 - offset };
}

struct Circle {
    float2 centre;
    float radius;
};
Circle operator+(const Circle& circle, const float2& offset) {
    return { circle.centre + offset,circle.radius };
}
Circle operator-(const Circle& circle, const float2& offset) {
    return { circle.centre - offset,circle.radius };
}

void LineAndCircleBoundedCollidable::doTickOfCollisions(){
    if (collidables.empty()) {
        return; // Need to ensure front() exists, also nothing to do if empty
    }

    while ((*(collidables.begin()))->timeOfCollision < 1) {
        LineAndCircleBoundedCollidable& first = **(collidables.begin());
        if (!first.nextPossibleCollision) { // No collision
            first.checkForNextCollision();
            continue;
        }
        LineAndCircleBoundedCollidable& other = *(first.nextPossibleCollision);

        // Step to collision
        if (first.timeOfCollision > first.timeAhead) { // Stop slightly short to avoid problems with rounding and intersecting slightly
            first.location += first.velocity * (first.timeOfCollision - first.timeAhead);
        }
        if (other.timeOfCollision > other.timeAhead) {
            other.location += other.velocity * (other.timeOfCollision - other.timeAhead);
        }
        first.timeAhead = first.timeOfCollision;
        other.timeAhead = other.timeOfCollision; // should be the same timeOfCollision

        float2& forceVec = first.forceVec;

        if (dotProduct(forceVec, forceVec) == 0) { // If forceVec is zero vector
            //forceVec = other.location - first.location; // Use an arbitrary direction. Will be buggy if this happens, but better than a crash
            throw "Force has no direction";
        }

        // Perpendicular part of bounce
        float X = -2 * dotProduct(first.velocity - other.velocity, forceVec)
            / dotProduct(forceVec, (first.getInverseMassMatrix() + other.getInverseMassMatrix()) * forceVec);
        X *= (1 + first.getCorFactorPerp()) / 2;
        X *= (1 + other.getCorFactorPerp()) / 2;

        if (!isnormal(X)) { // Check X was calculated fine
            //X = 0.0f; // This case is a problem
            throw "Cannot calculate new trajectories, X = " + std::to_string(X);
        }

        first.velocity += first.getInverseMassMatrix() * (X * forceVec);
        other.velocity -= other.getInverseMassMatrix() * (X * forceVec);

        // Tangential part of bounce
        float2 velocityInPlane1 = first.velocity - forceVec * dotProduct(first.velocity, forceVec) / dotProduct(forceVec, forceVec);
        float2 velocityInPlane2 = other.velocity - forceVec * dotProduct(other.velocity, forceVec) / dotProduct(forceVec, forceVec);
        float2 velDif = velocityInPlane2 - velocityInPlane1; // Direction of force
        float2 sampleVelChange1 = first.getInverseMassMatrix() * velDif;
        float2 sampleVelChange2 = -(other.getInverseMassMatrix() * velDif);
        // For CoR = 0:
        // velocityInPlane1 + x * sampleVelChange1 = velocityInPlane2 + x * sampleVelChange2
        // x * (sampleVelChange1 - sampleVelChange2) = velocityInPlane2 - velocityInPlane1
        // x = (velocityInPlane2 - velocityInPlane1).(sampleVelChange1 - sampleVelChange2) / |sampleVelChange1 - sampleVelChange2|^2
        // ...or velocityInPlane2 - velocityInPlane1 = 0
        float x = dotProduct(velDif, sampleVelChange1 - sampleVelChange2)
            / dotProduct(sampleVelChange1 - sampleVelChange2, sampleVelChange1 - sampleVelChange2);
        if (!isnan(x)) {
            float factor = 1.0f - first.getCorFactorTang() * other.getCorFactorTang();
            first.velocity += factor * x * sampleVelChange1;
            other.velocity += factor * x * sampleVelChange2;
        }

        first.onCollision();
        other.onCollision();
        first.checkForNextCollision();
        other.checkForNextCollision();
    }

    // Let everything finish its timestep
    // Reset timeAhead and decrease timeOfCollision by 1
    for (auto ptr : collidables) {
        ptr->location += ptr->velocity * (1 - ptr->timeAhead);
        ptr->timeAhead = 0.0f;
        if (ptr->timeOfCollision != INFINITY) // Check that there is a real collision. Probably not needed due to error in float at this size.
            ptr->timeOfCollision -= 1.0f; // Changing the multiset's sorting value, only okay because the order is the same at the end (all values > 1)
    }
}

LineAndCircleBoundedCollidable::LineAndCircleBoundedCollidable(const float2& initLocation, const float2& initVelocity)
    : location{ initLocation }, velocity{ initVelocity }, timeAhead{ 0.0f }, timeOfCollision{ 0.0f }, nextPossibleCollision{ nullptr }, forceVec{ 0.0f,0.0f }
{
    collidables.insert(this);
}

LineAndCircleBoundedCollidable::~LineAndCircleBoundedCollidable()
{
    // Remove self from collidables list
    collidables.erase(this);

    // Unpair
    if (nextPossibleCollision) {
        nextPossibleCollision->nextPossibleCollision = nullptr;
        nextPossibleCollision->forceVec = { 0.0f,0.0f };
    }
}

LineAndCircleBoundedCollidable::LineAndCircleBoundedCollidable(LineAndCircleBoundedCollidable&& other) noexcept
    : location{ other.location }, velocity{ other.velocity }, timeAhead{ other.timeAhead }, timeOfCollision{ 0.0f },
    nextPossibleCollision{ other.nextPossibleCollision }, lines{ std::move(other.lines) }, circles{ std::move(other.circles) }, forceVec{ 0.0f,0.0f }
{
    // Update pointer of paired object
    other.nextPossibleCollision = nullptr;
    if (nextPossibleCollision)
        nextPossibleCollision->nextPossibleCollision = this;

    // Add self to list
    collidables.insert(this);
    updateListPosition(other.timeOfCollision);
}

LineAndCircleBoundedCollidable& LineAndCircleBoundedCollidable::operator=(LineAndCircleBoundedCollidable&& other) noexcept
{
    // Unpair
    if (nextPossibleCollision) {
        nextPossibleCollision->nextPossibleCollision = nullptr;
        nextPossibleCollision->forceVec = { 0.0f,0.0f };
    }

    // Copy and move members
    location = other.location;
    velocity = other.velocity;
    timeAhead = other.timeAhead;
    nextPossibleCollision = other.nextPossibleCollision;
    forceVec = other.forceVec;
    lines = std::move(other.lines);
    circles = std::move(other.circles);

    // Update pointer of new paired object
    other.nextPossibleCollision = nullptr;
    if (nextPossibleCollision)
        nextPossibleCollision->nextPossibleCollision = this;

    // Move to correct point in list
    updateListPosition(other.timeOfCollision);

    return *this;
}

void LineAndCircleBoundedCollidable::changeTrajectory(const float2& newLocation, const float2& newVelocity) {
    location = newLocation;
    velocity = newVelocity;

    // Unpair
    if (nextPossibleCollision) {
        nextPossibleCollision->nextPossibleCollision = nullptr;
        nextPossibleCollision->forceVec = { 0.0f,0.0f };
    }
    nextPossibleCollision = nullptr;
    updateListPosition(timeAhead);
}

void LineAndCircleBoundedCollidable::changeVelocity(const float2& newVelocity) {
    changeTrajectory(location, newVelocity);
}

void LineAndCircleBoundedCollidable::addLine(const float2& p1, const float2& p2) {
    lines.emplace_back(Line{ p1,p2 });
    updateListPosition(timeAhead);
}

void LineAndCircleBoundedCollidable::addCircle(const float2& centre, float radius) {
    circles.emplace_back(Circle{ centre,radius });
    updateListPosition(timeAhead);
}

// Takes a line positioned relative to a point, and the velocity of the line relative to the point
// Returns the time that the line collides with the point
// Returns NaN if there is no collision
// Returns a negative number if the collision started/happened in the past
float pointLineTimeToCollision(const Line& line, const float2& relativeVelocity) {
    // Find whether the line will hit the origin or not, then find time.
    // Points on line can be written as: line.p1 + x * (line.p2 - line.p1), where 0 <= x <= 1
    // For point that hits the origin:
    // (lineP1 + x * (lineP2 - lineP1)).relativeVelocityPerp = 0
    // x = -lineP1.relativeVelocityPerp / (lineP2 - lineP1).relativeVelocityPerp
    // If 0 <= x <= 1, there is a hit. Else, miss
    float2 relativeVelocityPerp = { relativeVelocity.y,-relativeVelocity.x };
    float x = dotProduct(-line.p1, relativeVelocityPerp) / dotProduct((line.p2 - line.p1), relativeVelocityPerp);
    if (x < 0 || x > 1) {
        return NAN;
    }

    // Time of collision = -(lineP1 + x * (lineP2 - lineP1)).relativeVelocity / relativeVelocity.relativeVelocity
    return -dotProduct((line.p1 + x * (line.p2 - line.p1)), relativeVelocity) / dotProduct(relativeVelocity, relativeVelocity);
}

// Takes in two lines, and the velocity of the second relative to the first
// Returns the time at which the lines will begin to intersect
// Will return zero if the lines are currently intersecting, and are closer to when the intersection started than when it will finish
// Returns Inf is there is no collision, or if the normals are in the wrong direction
float timeToCollisionLines(const Line& a, const Line& b, const float2& relativeVelocity, float2* const forceVec = nullptr) { // Completely broken rn
    // Find time at which point intersects with parallelogram

    // Check that velocity is in direction of outwards line normal. If not, ignore collision.
    // (This is to handle the case of parallel lines that have managed to step past each other)
    if (dotProduct(relativeVelocity, { -(b.p2.y - b.p1.y),b.p2.x - b.p1.x }) < 0
        || dotProduct(relativeVelocity, { -(a.p2.y - a.p1.y),a.p2.x - a.p1.x }) > 0) {
        return INFINITY;
    }

    float earliestTime = INFINITY;
    float2 earliestForceVec = { 0.0f,0.0f };
    float minPosTime = INFINITY;
    float2 minPosForceVec = { 0.0f,0.0f };
    float time = pointLineTimeToCollision(Line{ b.p2 + a.p1 - a.p2 - a.p1,b.p2 - a.p1 }, relativeVelocity);
    if (!isnan(time)) { // If collision
        if (time < earliestTime) {
            earliestForceVec = { a.p1.y - a.p2.y,a.p2.x - a.p1.x }; // Perp to 'a'
            earliestTime = time;
        }
        if (0 <= time && time < minPosTime) { // If collision isn't in the past
            minPosForceVec = { a.p1.y - a.p2.y,a.p2.x - a.p1.x }; // Perp to 'a'
            minPosTime = time;
        }
    }
    time = pointLineTimeToCollision(Line{ b.p2 - a.p1,b.p1 - a.p1 }, relativeVelocity);
    if (!isnan(time)) { // If collision
        if (time < earliestTime) {
            earliestForceVec = { b.p1.y - b.p2.y,b.p2.x - b.p1.x }; // Perp to 'b'
            earliestTime = time;
        }
        if (0 <= time && time < minPosTime) { // If collision isn't in the past
            minPosForceVec = { b.p1.y - b.p2.y,b.p2.x - b.p1.x }; // Perp to 'b'
            minPosTime = time;
        }
    }
    time = pointLineTimeToCollision(Line{ b.p1 - a.p1,b.p1 + a.p1 - a.p2 - a.p1 }, relativeVelocity);
    if (!isnan(time)) { // If collision
        if (time < earliestTime) {
            earliestForceVec = { a.p1.y - a.p2.y,a.p2.x - a.p1.x }; // Perp to 'a'
            earliestTime = time;
        }
        if (0 <= time && time < minPosTime) { // If collision isn't in the past
            minPosForceVec = { a.p1.y - a.p2.y,a.p2.x - a.p1.x }; // Perp to 'a'
            minPosTime = time;
        }
    }
    time = pointLineTimeToCollision(Line{ b.p1 + a.p1 - a.p2 - a.p1,b.p2 + a.p1 - a.p2 - a.p1 }, relativeVelocity);
    if (!isnan(time)) { // If collision
        if (time < earliestTime) {
            earliestForceVec = { b.p1.y - b.p2.y,b.p2.x - b.p1.x }; // Perp to 'b'
            earliestTime = time;
        }
        if (0 <= time && time < minPosTime) { // If collision isn't in the past
            minPosForceVec = { b.p1.y - b.p2.y,b.p2.x - b.p1.x }; // Perp to 'b'
            minPosTime = time;
        }
    }
    if (isinf(minPosTime)) {
        if (forceVec)
            *forceVec = { 0.0f,0.0f };
        return INFINITY; // No collision in future
    }
    if (earliestTime <= 0) { // If also collisions happening in the past, then there must be an intersection (because combined shape is convex)
        // <= is needed above so that we can handle the case of a zero-thickness parallelogram here
        if (-earliestTime < minPosTime) { // If intersection closer to start than finish. Will fail if exactly halfway
            if (forceVec)
                *forceVec = earliestForceVec;
            return 0.0f; // Collide immediately
        }
        else { // If intersection closer to end than beginning
            if (forceVec)
                *forceVec = { 0.0f,0.0f };
            return INFINITY; // Let objects stop intersecting
        }
    }
    if (forceVec)
        *forceVec = minPosForceVec;
    return minPosTime;
}

// Returns the time that a point will collide with a circle
// Will return a negative number if collision would have started/happened in the past
// Will return NaN if there is no collision
// Takes the centre of the circle relative to the point, the radius of the circle, and the velocity of the circle relative to the point
float pointCircleTimeToCollision(const Circle& circle, const float2& relativeVelocity) {
    // Find whether the point will hit the circle or not, then find time.
    float2 relVelPerp = { relativeVelocity.y,-relativeVelocity.x };
    float perpDistanceTimesSpeed = dotProduct(circle.centre, relVelPerp); // Could try scaling vel and velperp vectors to avoid chance of speedSq = 0
    float speedSq = dotProduct(relativeVelocity, relativeVelocity);
    if (perpDistanceTimesSpeed * perpDistanceTimesSpeed >= circle.radius * circle.radius * speedSq) { // using >= handles case where relativeVelocity is 0
        return NAN; // Miss
    }

    // Distance from dead-on hit: x = abs(dotProduct(otherLoc - thisLoc, relVelPerp)) / sqrt(dotProduct(relVelPerp, relVelPerp))
    // Time to passing: - dotProduct(otherLoc - thisLoc,relativeVelocity) / dotProduct(relativeVelocity,relativeVelocity)
    // Shortening of time due to value of x: sqrt((this->radius + ptr->radius)^2 - x^2) / sqrt(dotProduct(relativeVelocity,relativeVelocity))
    // x^2 = perpDistanceTimesSpeed^2 / dotProduct(relVelPerp, relVelPerp)
    return -dotProduct(circle.centre, relativeVelocity) / speedSq
        - sqrt((circle.radius * circle.radius - perpDistanceTimesSpeed * perpDistanceTimesSpeed / speedSq) / speedSq);
}

// Takes a circle, a line, and the velocity of the line relative to the circle
// Returns the time that they collide
// Returns Inf if there is no collision
float timeToCollisionCircleLine(const Circle& circle, const Line& line, const float2& relativeVelocity, float2* const forceVec = nullptr) {
    // Check for collision of a point with c=) shape
    const float2 lineVec = line.p2 - line.p1;
    const float2 lineVecPerp = { lineVec.y,-lineVec.x };
    float earliestTime = INFINITY;
    float2 earliestForceVec = { 0.0f,0.0f };
    float minPosTime = INFINITY;
    float2 minPosForceVec = { 0.0f,0.0f };
    float time = pointLineTimeToCollision(line - circle.centre + lineVecPerp * circle.radius / sqrt(dotProduct(lineVecPerp, lineVecPerp)), relativeVelocity);
    if (!isnan(time)) { // If collision
        if (time < earliestTime) {
            earliestForceVec = { line.p1.y - line.p2.y,line.p2.x - line.p1.x }; // Perp to line
            earliestTime = time;
        }
        if (0 <= time && time < minPosTime) { // If collision isn't in the past
            minPosForceVec = { line.p1.y - line.p2.y,line.p2.x - line.p1.x }; // Perp to line
            minPosTime = time;
        }
    }
    time = pointLineTimeToCollision(line - circle.centre - lineVecPerp * circle.radius / sqrt(dotProduct(lineVecPerp, lineVecPerp)), relativeVelocity);
    if (!isnan(time)) { // If collision
        if (time < earliestTime) {
            earliestForceVec = { line.p1.y - line.p2.y,line.p2.x - line.p1.x }; // Perp to line
            earliestTime = time;
        }
        if (0 <= time && time < minPosTime) { // If collision isn't in the past
            minPosForceVec = { line.p1.y - line.p2.y,line.p2.x - line.p1.x }; // Perp to line
            minPosTime = time;
        }
    }
    time = pointCircleTimeToCollision(circle - line.p1, -relativeVelocity);
    if (!isnan(time)) { // If collision
        if (time < earliestTime) {
            earliestForceVec = circle.centre - line.p1 - relativeVelocity * time; // Location of centre of circle at time of collision, relative to p1
            earliestTime = time;
        }
        if (0 <= time && time < minPosTime) { // If collision isn't in the past
            minPosForceVec = circle.centre - line.p1 - relativeVelocity * time; // Location of centre of circle at time of collision, relative to p1
            minPosTime = time;
        }
    }
    time = pointCircleTimeToCollision(circle - line.p2, -relativeVelocity);
    if (!isnan(time)) { // If collision
        if (time < earliestTime) {
            earliestForceVec = circle.centre - line.p2 - relativeVelocity * time; // Location of centre of circle at time of collision, relative to p2
            earliestTime = time;
        }
        if (0 <= time && time < minPosTime) { // If collision isn't in the past
            minPosForceVec = circle.centre - line.p2 - relativeVelocity * time; // Location of centre of circle at time of collision, relative to p2
            minPosTime = time;
        }
    }
    if (isinf(minPosTime)) {
        if (forceVec)
            *forceVec = { 0.0f,0.0f };
        return INFINITY; // No collision in future
    }
    if (earliestTime < 0) { // If also collisions happening in the past, then there must be an intersection (because combined shape is convex)
        if (-earliestTime < minPosTime) { // If intersection closer to start than finish
            if (forceVec)
                *forceVec = earliestForceVec;
            return 0.0f; // Collide immediately
        }
        else { // If intersection closer to end than beginning
            if (forceVec)
                *forceVec = { 0.0f,0.0f };
            return INFINITY; // Let objects stop intersecting
        }
    }
    if (forceVec)
        *forceVec = minPosForceVec;
    return minPosTime;
}

// Takes two circles, and the velocity of the second relative to the first
// Returns the time that they collide.
// Returns Inf if there is no collision
float timeToCollisionCircles(const Circle& a, const Circle& b, const float2& relativeVelocity, float2* const forceVec = nullptr) {
    float time = pointCircleTimeToCollision(Circle{ b.centre - a.centre ,a.radius + b.radius }, relativeVelocity);
    if (isnan(time)) {
        if (forceVec)
            *forceVec = { 0.0f,0.0f };
        return INFINITY; // No collision
    }
    if (forceVec)
        *forceVec = b.centre - a.centre + relativeVelocity * time;
    if (time >= 0.0f)
        return time;
    // Need to check if objects have intersected slightly, or are just moving apart
    float timeReverse = pointCircleTimeToCollision(Circle{ b.centre - a.centre ,a.radius + b.radius }, -relativeVelocity);
    if (timeReverse > time) {
        if (forceVec)
            *forceVec = { 0.0f,0.0f };
        return INFINITY; // Either moving apart, or closer to the end of the intersection than the start
    }
    else {
        return 0.0f; // Just started intersecting, should collide instantly
    }
}

void LineAndCircleBoundedCollidable::checkForNextCollision() {
    // Find when next collision will be, if everything stays on current trajectories
    
    // If there is a current possible collision, then the other object needs to have it's pointer made null. This shouldn't be needed?
    if (nextPossibleCollision) { // If paired with something
        nextPossibleCollision->nextPossibleCollision = nullptr; // Unpair
        nextPossibleCollision->forceVec = { 0.0f,0.0f };
    }

    float newTimeOfCollision = INFINITY;
    nextPossibleCollision = nullptr;
    forceVec = { 0.0f,0.0f };
    for (auto other : collidables) {
        // Synchronise objects
        float2 thisLoc = this->location;
        float2 otherLoc = other->location;
        float thisTA = this->timeAhead;
        float otherTA = other->timeAhead;
        if (thisTA < otherTA) { // Advance this->location
            thisLoc += (otherTA - thisTA) * this->velocity;
            thisTA = otherTA;
        }
        else { // Advance other->location
            otherLoc += (thisTA - otherTA) * other->velocity;
            otherTA = thisTA; // Not actually used
        }
        float2 relativeVelocity = other->velocity - this->velocity;

        float minTime = INFINITY;
        float2 forceVecTemp;
        float2 thisCollisionForceVec = { 0.0f,0.0f };
        for (auto& line : this->lines) {
            for (auto& line2 : other->lines) {
                float time = timeToCollisionLines(line + thisLoc, line2 + otherLoc, relativeVelocity, &forceVecTemp);
                if (time < minTime) {
                    minTime = time;
                    thisCollisionForceVec = forceVecTemp;
                }
            }
            for (auto& circle : other->circles) {
                float time = timeToCollisionCircleLine(circle + otherLoc, line + thisLoc, -relativeVelocity, &forceVecTemp);
                if (time < minTime) {
                    minTime = time;
                    thisCollisionForceVec = forceVecTemp;
                }
            }
        }
        for (auto& circle : this->circles) {
            for (auto& line : other->lines) {
                float time = timeToCollisionCircleLine(circle + thisLoc, line + otherLoc, relativeVelocity, &forceVecTemp);
                if (time < minTime) {
                    minTime = time;
                    thisCollisionForceVec = forceVecTemp;
                }
            }
            for (auto& circle2 : other->circles) {
                float time = timeToCollisionCircles(circle + thisLoc, circle2 + otherLoc, relativeVelocity, &forceVecTemp);
                if (time < minTime) {
                    minTime = time;
                    thisCollisionForceVec = forceVecTemp;
                }
            }
        }
        minTime += thisTA;
        if (minTime < newTimeOfCollision && minTime < other->timeOfCollision) {
            newTimeOfCollision = minTime;
            nextPossibleCollision = other;
            forceVec = thisCollisionForceVec;
        }
    }

    // Move to new position in list
    updateListPosition(newTimeOfCollision);
    if (nextPossibleCollision) {
        if (nextPossibleCollision->nextPossibleCollision) // If it is paired
        {
            nextPossibleCollision->nextPossibleCollision->nextPossibleCollision = nullptr; // Unpair
            nextPossibleCollision->nextPossibleCollision->forceVec = { 0.0f,0.0f };
        }
        nextPossibleCollision->nextPossibleCollision = this; // Is 'this' correct? Make sure
        nextPossibleCollision->forceVec = forceVec;
        nextPossibleCollision->updateListPosition(newTimeOfCollision);
    }
}

void LineAndCircleBoundedCollidable::updateListPosition(float newTimeOfCollision) {
    auto nodeHandle = collidables.extract(this);
    timeOfCollision = newTimeOfCollision;
    collidables.insert(std::move(nodeHandle));
}

bool LineAndCircleBoundedCollidable::comparisonFunction::operator()(const LineAndCircleBoundedCollidable* const a, const LineAndCircleBoundedCollidable* const b)const
{
    if (a->timeOfCollision < b->timeOfCollision)
        return true;
    if (a->timeOfCollision > b->timeOfCollision)
        return false;
    return a < b; // Can't have two different objects treated as equivalent
}
