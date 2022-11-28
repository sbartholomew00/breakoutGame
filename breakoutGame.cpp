#include "displaySystem.h"
#include "LineAndCircleBoundedCollidable.h"
#include <assert.h>
#include <vector>
#include <list>

const LPCWSTR propName = L"BreakoutGame";

struct Rect {
    float x;
    float y;
    float w;
    float h;
};

Rect getRect(int gridSizeX, int gridSizeY, int gridIndX, int gridIndY, float scale) {
    Rect retRect;
    retRect.w = 2.0f / gridSizeX;
    retRect.h = 2.0f / gridSizeY;
    retRect.x = -1.0f + gridIndX * retRect.w;
    retRect.y = 1.0f - gridIndY * retRect.h;
    retRect.x += retRect.w * (1 - scale) / 2;
    retRect.y -= retRect.h * (1 - scale) / 2;
    retRect.w *= scale;
    retRect.h *= scale;
    return retRect;
}

Rect getRect(int gridSizeX, int gridSizeY, int gridIndX, int gridIndY, float scale, Rect gridRect) {
    Rect retRect;
    retRect.w = gridRect.w / gridSizeX;
    retRect.h = gridRect.h / gridSizeY;
    retRect.x = gridRect.x + gridIndX * retRect.w;
    retRect.y = gridRect.y - gridIndY * retRect.h;
    retRect.x += retRect.w * (1 - scale) / 2;
    retRect.y -= retRect.h * (1 - scale) / 2;
    retRect.w *= scale;
    retRect.h *= scale;
    return retRect;
}

class CircleObject : private LineAndCircleBoundedCollidable {
    float radius;

    CircleObject& operator=(CircleObject&&) = delete;
    CircleObject(const CircleObject&) = delete;
    CircleObject& operator=(const CircleObject&) = delete;
public:
    using LineAndCircleBoundedCollidable::getLocation;
    using LineAndCircleBoundedCollidable::getVelocity;
    using LineAndCircleBoundedCollidable::changeTrajectory;
    using LineAndCircleBoundedCollidable::changeVelocity;

    CircleObject(float2 location, float2 velocity, float initRadius, float mass) : LineAndCircleBoundedCollidable{ location, velocity }, radius{ initRadius } {
        /*// For octagon with diagonal = 2 * radius:
        // Diameter = side length * sqrt(4 + 2sqrt(2))
        // Area = 2 * side length^2 * (1 + sqrt(2))
        // Side length = diameter / sqrt(4 + 2sqrt(2))
        float sideLength = 2 * radius / sqrt(4 + 2 * sqrt(2));
        // SmallRadius^2 = radius^2 - (side length/2)^2
        float smallRadius = sqrt(radius * radius - sideLength * sideLength / 4);
        addLine({ -sideLength / 2,smallRadius }, { sideLength / 2,smallRadius });
        addLine({ sideLength / 2,smallRadius }, { smallRadius,sideLength / 2 });
        addLine({ smallRadius,sideLength / 2 }, { smallRadius,-sideLength / 2 });
        addLine({ smallRadius,-sideLength / 2 }, { sideLength / 2,-smallRadius });
        addLine({ sideLength / 2,-smallRadius }, { -sideLength / 2,-smallRadius });
        addLine({ -sideLength / 2,-smallRadius }, { -smallRadius,-sideLength / 2 });
        addLine({ -smallRadius,-sideLength / 2 }, { -smallRadius,sideLength / 2 });
        addLine({ -smallRadius,sideLength / 2 }, { -sideLength / 2,smallRadius });*/

        /*// For Hexagon
        addLine({-sideLength / 2,smallRadius}, {sideLength / 2,smallRadius});
        addLine({ sideLength / 2,smallRadius }, { radius,0.0f });
        addLine({ radius,0.0f }, { sideLength / 2,-smallRadius });
        addLine({ sideLength / 2,-smallRadius }, { -sideLength / 2,-smallRadius });
        addLine({ -sideLength / 2,-smallRadius }, { -radius,0.0f });
        addLine({ -radius,0.0f }, { -sideLength / 2,smallRadius });*/

        addCircle({ 0.0f,0.0f }, radius);
    }

    CircleObject(CircleObject&& other) noexcept : LineAndCircleBoundedCollidable{ std::move(other) }, radius{ other.radius } {}
};

class RectangularObject : private LineAndCircleBoundedCollidable {
    RectangularObject& operator=(RectangularObject&&) = delete;
    RectangularObject(const RectangularObject&) = delete;
    RectangularObject& operator=(const RectangularObject&) = delete;

public:
    using LineAndCircleBoundedCollidable::getLocation;
    using LineAndCircleBoundedCollidable::getVelocity;
    using LineAndCircleBoundedCollidable::changeTrajectory;
    using LineAndCircleBoundedCollidable::changeVelocity;

    RectangularObject(Rect rect, float2 velocity) : LineAndCircleBoundedCollidable{ float2{ rect.x,rect.y }, velocity } {
        addLine({ 0,0 }, { rect.w,0 });
        addLine({ rect.w,0 }, { rect.w,-rect.h });
        addLine({ rect.w,-rect.h }, { 0,-rect.h });
        addLine({ 0,-rect.h }, { 0,0 });
    }

    RectangularObject(RectangularObject&& other) noexcept : LineAndCircleBoundedCollidable{ std::move(other) } {}
};

class Block : private RectangularObject {
    DisplaySystem::VisualComponent image;
    unsigned int health;

    Block& operator=(Block&&) = delete;
    Block(const Block&) = delete;
    Block& operator=(const Block&) = delete;
    virtual const Matrix2x2 getInverseMassMatrix() {
        return { 0,0,0,0 };
    }
    virtual void onCollision() {
        if (health)
            --health;
        // Could change image
    }
public:
    Block(Rect rect, unsigned initHealth)
        : image{ "images/Block.png",rect.x,rect.y,rect.w,rect.h }, health{ initHealth }, RectangularObject{ rect,{0.0f,0.0f} } {}
    // Returns true if should be destroyed
    bool tick() {
        if (!health) {
            return true;
        }
        return false;
    }

    Block(Block&& other) noexcept : RectangularObject{ std::move(other) }, image{ std::move(other.image) }, health{ other.health } {}
};

class Wall : private RectangularObject {
    DisplaySystem::VisualComponent image;

    Wall& operator=(Wall&&) = delete;
    Wall(const Wall&) = delete;
    Wall& operator=(const Wall&) = delete;
    virtual const Matrix2x2 getInverseMassMatrix() {
        return { 0,0,0,0 };
    }
public:
    Wall(Rect rect) : image{ "images/Wall.bmp",rect.x,rect.y,rect.w,rect.h }, RectangularObject{ rect,{0.0f,0.0f} } {}

    Wall(Wall&& other) noexcept : image{ std::move(other.image) }, RectangularObject{ std::move(other) }{}
};

class Ball : private CircleObject {
    DisplaySystem::VisualComponent image;
    float radius;
    float mass;

    Ball& operator=(Ball&&) = delete;
    Ball(const Ball&) = delete;
    Ball& operator=(const Ball&) = delete;
    virtual float getCorFactorPerp() {
        return 1.0f;
    }
    virtual const Matrix2x2 getInverseMassMatrix() {
        return { 1 / mass,0,0,1 / mass };
    }
public:
    Ball(float2 location, float2 velocity, float initRadius, float initMass)
        : image{ "images/Ball.png",location.x - initRadius,location.y + initRadius,2 * initRadius,2 * initRadius },
        CircleObject{ location,velocity,initRadius,initMass }, radius{ initRadius }, mass{ initMass } {}
    bool isOffScreen() {
        float2 loc = getLocation();
        return max(abs(loc.x), abs(loc.y)) > 1.0f + radius;
    }
    bool tick() {
        float2 loc = getLocation();
        image.changeLocation(loc.x - radius, loc.y + radius);
        return isOffScreen();
    }

    Ball(Ball&& other) noexcept : image{ std::move(other.image) }, CircleObject{ std::move(other) }, radius{ other.radius }, mass{ other.mass } {}
};

class Bat : private LineAndCircleBoundedCollidable {
    DisplaySystem::VisualComponent leftBat;
    DisplaySystem::VisualComponent centreBat;
    DisplaySystem::VisualComponent rightBat;
    float width;
    float height;
    bool movingLeft;
    bool movingRight;
    static constexpr float mass = 1.0f;

    Bat(Bat&&) = delete;
    Bat& operator=(Bat&&) = delete;
    Bat(const Bat&) = delete;
    Bat& operator=(const Bat&) = delete;

    virtual float getCorFactorPerp() {
        return 1.0f;
    }
    virtual float getCorFactorTang() {
        return 0.5f; // make init redundant by doing collision checks before while loop. replace collidable with lineAndCircleBoundedColliadable
    }
    virtual const Matrix2x2 getInverseMassMatrix() {
        return { 1 / mass,0,0,0 };
    }
    virtual void onCollision() {
        if (getVelocity() != float2{ 0.0f, 0.0f })
            changeVelocity({ 0.0f,0.0f });
    }
public:
    Bat(Rect rect) : leftBat{ "images/LeftBat.png",rect.x,rect.y,rect.h / 2.0f,rect.h },
        centreBat{ "images/BatCentre.png",rect.x + rect.h / 2.0f,rect.y,rect.w - rect.h,rect.h },
        rightBat{ "images/RightBat.png",rect.x + rect.w - rect.h / 2.0f,rect.y,rect.h / 2.0f,rect.h },
        width{ rect.w }, height{ rect.h },
        LineAndCircleBoundedCollidable{ {rect.x,rect.y},{0.0f,0.0f} },
        movingLeft{ false }, movingRight{ false } {
        addCircle({ rect.h / 2,-rect.h / 2 }, rect.h / 2);
        addCircle({ rect.w - rect.h / 2,-rect.h / 2 }, rect.h / 2);
        addLine({ rect.h / 2,0.0f }, { rect.w - rect.h / 2,0.0f });
        addLine({ rect.w - rect.h / 2, -rect.h }, { rect.h / 2, -rect.h });
    }
    void tick() {
        if (movingLeft == movingRight) {
            if (getVelocity() != float2{ 0.0f, 0.0f })
                changeVelocity({ 0.0f,0.0f });
        }
        else {
            if (movingLeft) {
                if (getVelocity() != float2{ -0.03f,0.0f })
                    changeVelocity({ -0.03f,0.0f });
            }
            else {
                if (getVelocity() != float2{ 0.03f,0.0f })
                    changeVelocity({ 0.03f,0.0f });
            }
        }
        float2 loc = getLocation();
        leftBat.changeLocation(loc.x, loc.y);
        centreBat.changeLocation(loc.x + height / 2.0f, loc.y);
        rightBat.changeLocation(loc.x + width - height / 2.0f, loc.y);
        movingLeft = false;
        movingRight = false;
    }
    void moveLeft() {
        movingLeft = true;
    }
    void moveRight() {
        movingRight = true;
    }
};

class BreakoutGame {
    std::vector<Wall> walls;
    std::list<Block> blocks;
    std::list<Ball> balls;
    Bat bat;
    bool leftDown;
    bool rightDown;
    bool leftUp;
    bool rightUp;
public:
    BreakoutGame()
        : bat{ Rect{ -0.1f,-0.84f,0.2f,0.05f } } {
        // Set images for display, destroy before cleanup
        for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < 8; ++j) {
                blocks.emplace_back(getRect(10, 20, i, 2 + j, 0.9f, { -0.9f,0.9f,1.8f,1.8f }), 1);
            }
        }
        balls.emplace_back(float2{ 0.0f,-0.5f }, float2{ -0.01f,-0.01f }, 0.025f, 1.0f);
        leftDown = false;
        rightDown = false;
        leftUp = false;
        rightUp = false;
        Rect temp = getRect(20, 1, 0, 0, 1.0f);
        walls.emplace_back(getRect(20, 1, 0, 0, 1.0f));
        walls.emplace_back(getRect(20, 1, 19, 0, 1.0f));
        walls.emplace_back(getRect(1, 20, 0, 0, 1.0f, { temp.x + temp.w,1.0f ,2.0f * 18.0f / 20.0f,2.0f }));
    }
    void tick() {
        if (leftDown)
            bat.moveLeft();
        if (rightDown)
            bat.moveRight();

        // Call tick() functions
        for (auto it = blocks.begin(); it != blocks.end();) {
            if (it->tick()) {
                it = blocks.erase(it);
            }
            else {
                ++it;
            }
        }
        for (auto it = balls.begin(); it != balls.end();) {
            if (it->tick()) {
                it = balls.erase(it);
            }
            else {
                ++it;
            }
        }
        bat.tick();

        // If no blocks or no balls
        if (blocks.empty() || balls.empty()) {
            // Reset
            blocks.clear();
            balls.clear();

            for (int i = 0; i < 10; ++i) {
                for (int j = 0; j < 8; ++j) {
                    blocks.emplace_back(getRect(10, 20, i, 2 + j, 0.9f, { -0.9f,0.9f,1.8f,1.8f }), 1);
                }
            }
            balls.emplace_back(float2{ 0.0f,-0.5f }, float2{ -0.01f,-0.01f }, 0.025f, 1.0f);
        }

        // Update screen
        DisplaySystem::update();

        // Do collisions
        LineAndCircleBoundedCollidable::doTickOfCollisions();

        // Reset key states
        if (leftUp)
            leftDown = false;
        if (rightUp)
            rightDown = false;
        leftUp = false;
        rightUp = false;
    }
    void leftPress() {
        leftDown = true;
    }
    void rightPress() {
        rightDown = true;
    }
    void leftReleased() {
        leftUp = true;
    }
    void rightReleased() {
        rightUp = true;
    }
};

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg)
    {
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    case WM_KEYDOWN:
        if (!(HIWORD(lParam) & KF_REPEAT)) {
            switch (wParam) {
            case VK_ESCAPE:
                PostQuitMessage(0);
                return 0;
            case 'A':
                static_cast<BreakoutGame*>(GetProp(hwnd, propName))->leftPress();
                return 0;
            case 'D':
                static_cast<BreakoutGame*>(GetProp(hwnd, propName))->rightPress();
                return 0;
            }
        }
        break;
    case WM_KEYUP:
        switch (wParam) {
        case 'A':
            static_cast<BreakoutGame*>(GetProp(hwnd, propName))->leftReleased();
            return 0;
        case 'D':
            static_cast<BreakoutGame*>(GetProp(hwnd, propName))->rightReleased();
            return 0;
        }
        break;
    case WM_TIMER:
        // wParam = timer ID
        // lParam = callback if applicable
        if (wParam == 1) {
            static_cast<BreakoutGame*>(GetProp(hwnd, propName))->tick();
        }
        return 0;
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);   // Default message handling
}

int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow) {
    const int windowWidth = 500;
    const int windowHeight = 500;
    const wchar_t* windowName = L"breakoutGame";
    
    // Create window class
	WNDCLASS windowClass = {};
	windowClass.lpfnWndProc = WindowProc;
	windowClass.hInstance = hInstance;
	windowClass.lpszClassName = L"breakoutGameWindowClass";
    RegisterClass(&windowClass);

    // Find size of total window area for the given client area
    RECT rect;
    rect.left = 0;
    rect.top = 0;
    rect.right = windowWidth;
    rect.bottom = windowHeight;
    AdjustWindowRect(&rect, CS_OWNDC | WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU, false);

    // Create window
	HWND windowHandle = CreateWindowEx(
		0,
		windowClass.lpszClassName,
        windowName,
        CS_OWNDC | WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU, // Non-resizable
        CW_USEDEFAULT, CW_USEDEFAULT,   // Window location
        rect.right - rect.left, rect.bottom - rect.top,      // Window dimensions
        NULL, NULL,
        hInstance,
        NULL
		);
    assert(windowHandle != NULL);
    ShowWindow(windowHandle, nCmdShow);

    // Create OpenGL context
    DisplaySystem::init(windowHandle, 0, 0, windowWidth, windowHeight);
    {
        BreakoutGame game{};
        SetProp(windowHandle, propName, static_cast<HANDLE>(&game)); // So that callback can use game's methods

        // Timer for framerate
        SetTimer(windowHandle, 1, 1000 / 60, NULL); // Timer 1, 16ms/frame

        // Message loop
        MSG msg = {};
        while (GetMessage(&msg, NULL, 0, 0) > 0) {
            //TranslateMessage(&msg);   // Turns key presses into chars
            DispatchMessage(&msg);
        }

        // Stop timer
        KillTimer(windowHandle, 1);
        RemoveProp(windowHandle, propName);
    }
    // Check for errors
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        printf("");
    }
    // Delete OpenGL context
    DisplaySystem::cleanup();

	return 0;
}

void testDisplaySystem() {
    // Set images for display, destroy before cleanup
    std::vector<DisplaySystem::VisualComponent> comps;
    comps.emplace_back("images/testImage.bmp", -0.4f, -0.4f, 0.2f, 0.1f);
    comps.emplace_back("images/testImage.bmp", 0.4f, -0.4f, 0.1f, 0.2f);
    comps.emplace_back("images/testImage2.bmp", 0.0f, 0.4f, 0.2f, 0.2f);

    comps[1].changeLocation(0.4f, -0.5f);
    comps[1].changeImage("images/testImage2.bmp");
    comps[1].change("images/testImage.bmp", 0.4f, -0.3f, 0.1f, 0.2f);
    comps[1].changeImage("images/testImage2.bmp");

    comps.erase(comps.begin() + 1);
    comps[1].changeLocation(0.4f, -0.5f, 0.1f, 1.0f);
    comps[1].changeImage("images/testImage.bmp");
    comps.emplace_back("images/testImage2.bmp", 0.0f, 0.4f, 0.1f, 0.2f);
    comps.emplace_back("images/testImage.bmp", 0.4f, -0.4f, 0.3f, 0.2f);


    // Message loop
    MSG msg = {};
    while (GetMessage(&msg, NULL, 0, 0) > 0) {
        //TranslateMessage(&msg);   // Turns key presses into chars
        DispatchMessage(&msg);
    }

    comps.clear();
}