#pragma once
#include <string>
#include <Windows.h>	// Only needed for HWND typedef
#include <glad/glad.h>	// Only needed for GLint and GLsizei typedef

// Used to display 2D graphics
namespace DisplaySystem {

	// Updates the display to show all of the current VisualComponents
	void update();

	// Needs to be run before anything in DisplaySystem is used
	void init(HWND windowHandle, GLint x, GLint y, GLsizei windowWidth, GLsizei windowHeight);

	// Should be run before the end of the program. All VisualComponents should be destroyed before this.
	void cleanup();

	// An image displayed on the screen at a location.
	class VisualComponent {
		std::string imagePath;
		float xLoc;
		float yLoc;
		float wDim;
		float hDim;
		unsigned int instanceID;
		VisualComponent(const VisualComponent&) = delete;
		VisualComponent& operator=(const VisualComponent&) = delete;
	public:
		// Displays the image given by 'imagePath' at a top-left-corner location (x,y), width 'w' and height 'h'. Uses display normalised units
		VisualComponent(std::string imagePath, float x, float y, float w, float h);

		// Move constructor
		VisualComponent(VisualComponent&&) noexcept;

		// Move assignment
		VisualComponent& operator=(VisualComponent&&) noexcept;

		// Removes the image from the screen
		~VisualComponent();

		// Removes the current image and creates a new one. Params are the same as in the constructor
		void change(std::string imagePath, float x, float y, float w, float h);

		// Changes the top-left-corner location of the image
		void changeLocation(float x, float y);

		// Changes the top-left-corner location of the image, and its dimensions
		void changeLocation(float x, float y, float w, float h);

		// Replaces the current image with a new image, with the same location and dimensions
		void changeImage(std::string imagePath);
	};
}