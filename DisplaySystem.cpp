#include <unordered_map>
#include <glad/glad.h> 
#include "displaySystem.h"
#include "stb_image.h"
#include <assert.h>
#include <Windows.h>

namespace DisplaySystem {

    static GLuint shaderProgram;
    static HDC deviceContext;
    static HGLRC context;

    class Sprite;
    static std::unordered_map<std::string, Sprite> sprites;

    /// Manages an openGL buffer with data for where to display instances of an image.
    class Sprite {
        GLuint bufferID;
        std::unordered_map<unsigned int, unsigned int> indexFromID;
        std::unordered_map<unsigned int, unsigned int> indexToID;
        unsigned int allocatedSize;
        unsigned int size;
        unsigned int nextID;
        GLuint textureID;
        GLuint vaoID;
        int bindingIndex;
        Sprite(const Sprite&) = delete;
        Sprite& operator=(const Sprite&) = delete;
        Sprite& operator=(Sprite&&) = delete;
    public:
        /// 'filename' is the path to the image which this object will use
        Sprite(std::string filename) {
            glCreateBuffers(1, &bufferID);
            allocatedSize = 1;
            size = 0;
            glNamedBufferData(bufferID, allocatedSize * 4 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
            nextID = 1;

            // Load image to get data
            int width, height, nrChannels;
            unsigned char* data = stbi_load(filename.c_str(), &width, &height, &nrChannels, 0);
            assert(data != NULL);

            glCreateTextures(GL_TEXTURE_2D, 1, &textureID);
            glTextureStorage2D(textureID, 1, GL_RGB8, width, height);

            glTextureParameteri(textureID, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTextureParameteri(textureID, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTextureParameteri(textureID, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTextureParameteri(textureID, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

            if (nrChannels == 3)
                glTextureSubImage2D(textureID, 0, 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
            else if (nrChannels == 4)
                glTextureSubImage2D(textureID, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, data);
            glGenerateTextureMipmap(textureID);
            stbi_image_free(data);

            // Make VAO and set it up
            glCreateVertexArrays(1, &vaoID);
            int positionLocation = 0;
            int dimLocation = 1;
            bindingIndex = 0;
            glEnableVertexArrayAttrib(vaoID, positionLocation);
            glEnableVertexArrayAttrib(vaoID, dimLocation);
            glVertexArrayVertexBuffer(vaoID, bindingIndex, bufferID, 0, 4 * sizeof(float));
            glVertexArrayAttribFormat(vaoID, positionLocation, 2, GL_FLOAT, GL_FALSE, 0);
            glVertexArrayAttribFormat(vaoID, dimLocation, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float));
            glVertexArrayAttribBinding(vaoID, positionLocation, bindingIndex);
            glVertexArrayAttribBinding(vaoID, dimLocation, bindingIndex);
        }

        ~Sprite() {
            if (bufferID)
                glDeleteBuffers(1, &bufferID);
            if (textureID)
                glDeleteTextures(1, &textureID);
            if (vaoID)
                glDeleteVertexArrays(1, &vaoID);
        }

        Sprite(Sprite&& other) noexcept {
            bufferID = other.bufferID;
            allocatedSize = other.allocatedSize;
            size = other.size;
            nextID = other.nextID;
            textureID = other.textureID;
            vaoID = other.vaoID;
            bindingIndex = other.bindingIndex;
            indexFromID = std::move(other.indexFromID);
            indexToID = std::move(other.indexToID);
            other.bufferID = NULL;
            other.textureID = NULL;
            other.vaoID = NULL;
        }
        // Adds a new location on the screen at which the Sprite should be displayed
        // 'x' and 'y' are the position for the top left corner of the image, in display normalised units
        // 'w' and 'h' are the width and height of the image, in display normalised units
        // Returns an id which can be used to change/remove the element
        unsigned int addElement(float x, float y, float w, float h) {
            if (allocatedSize <= size) {
                GLuint newBufferID;
                glCreateBuffers(1, &newBufferID);
                glNamedBufferData(newBufferID, 2 * allocatedSize * 4 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
                glCopyNamedBufferSubData(bufferID, newBufferID, 0, 0, allocatedSize * 4 * sizeof(float));
                glDeleteBuffers(1, &bufferID);
                bufferID = newBufferID;
                glVertexArrayVertexBuffer(vaoID, bindingIndex, bufferID, 0, 4 * sizeof(float));
                allocatedSize *= 2;
            }
            float* ptr = static_cast<float*>(glMapNamedBuffer(bufferID, GL_WRITE_ONLY));
            *(ptr + size * 4) = x;
            *(ptr + size * 4 + 1) = y;
            *(ptr + size * 4 + 2) = w;
            *(ptr + size * 4 + 3) = h;
            glUnmapNamedBuffer(bufferID);
            // TODO: check if nextID is already a key
            indexFromID.insert({ nextID,size });
            indexToID.insert({ size,nextID });
            ++size;
            return nextID++; // nextID must not overflow, but it will eventually
        }
        // Removes an element from the buffer, so that it will not be displayed anymore.
        // 'id' is the value returned by addElement()
        void removeElement(unsigned int id) {
            unsigned int index = indexFromID[id];
            float* ptr = static_cast<float*>(glMapNamedBuffer(bufferID, GL_READ_WRITE));
            *(ptr + 4 * index) = *(ptr + (size - 1) * 4);
            *(ptr + 4 * index + 1) = *(ptr + (size - 1) * 4 + 1);
            *(ptr + 4 * index + 2) = *(ptr + (size - 1) * 4 + 2);
            *(ptr + 4 * index + 3) = *(ptr + (size - 1) * 4 + 3);
            glUnmapNamedBuffer(bufferID);
            int endID = indexToID[(size - 1)];
            indexFromID[endID] = index;
            indexToID[index] = endID;
            indexFromID.erase(id);
            indexToID.erase((size - 1));
            --size;
        }
        // Changes the location at which an image in displayed
        // 'id' is the value returned by addElement()
        void changeElement(unsigned int id, float x, float y, float w, float h) {
            unsigned int index = indexFromID[id];
            float* ptr = static_cast<float*>(glMapNamedBuffer(bufferID, GL_WRITE_ONLY));
            *(ptr + 4 * index) = x;
            *(ptr + 4 * index + 1) = y;
            *(ptr + 4 * index + 2) = w;
            *(ptr + 4 * index + 3) = h;
            glUnmapNamedBuffer(bufferID);
        }
        // Draws the image onto the screen at all the locations given in the buffer. Needs correct shader program to be bound
        void draw() {
            // Could record and rebind previously bound texture and vao to improve code safety
            glBindTexture(GL_TEXTURE_2D, textureID);
            glBindVertexArray(vaoID);
            glDrawArrays(GL_POINTS, 0, size);

            glBindVertexArray(NULL);
            glBindTexture(GL_TEXTURE_2D, NULL);
        }
    };

    VisualComponent::VisualComponent(std::string filename, float x, float y, float w, float h)
    {
        //instanceID = (sprites.try_emplace(filename, filename)).first->second.addElement(x, y, w, h); // One line option
        sprites.try_emplace(filename, filename);
        instanceID = sprites.at(filename).addElement(x, y, w, h);
        imagePath = filename;
        xLoc = x;
        yLoc = y;
        wDim = w;
        hDim = h;
    }

    VisualComponent::~VisualComponent() {
        if (instanceID && sprites.count(imagePath)) // Checks that sprites object hasn't been cleared in cleanup(). Could skip this to make it faster
            sprites.at(imagePath).removeElement(instanceID);
    }

    void VisualComponent::changeLocation(float x, float y) {
        sprites.at(imagePath).changeElement(instanceID, x, y, wDim, hDim);
        xLoc = x;
        yLoc = y;
    }

    void VisualComponent::changeLocation(float x, float y, float w, float h) {
        sprites.at(imagePath).changeElement(instanceID, x, y, w, h);
        xLoc = x;
        yLoc = y;
        wDim = w;
        hDim = h;
    }

    void VisualComponent::changeImage(std::string filename) {
        sprites.at(imagePath).removeElement(instanceID);

        sprites.try_emplace(filename, Sprite(filename));
        instanceID = sprites.at(filename).addElement(xLoc, yLoc, wDim, hDim);
        imagePath = filename;
    }

    void VisualComponent::change(std::string filename, float x, float y, float w, float h) {
        sprites.at(imagePath).removeElement(instanceID);

        sprites.try_emplace(filename, Sprite(filename));
        instanceID = sprites.at(filename).addElement(x, y, w, h);
        imagePath = filename;
        xLoc = x;
        yLoc = y;
        wDim = w;
        hDim = h;
    }

    VisualComponent::VisualComponent(VisualComponent&& other) noexcept {
        instanceID = other.instanceID;
        other.instanceID = NULL;
        xLoc = other.xLoc;
        yLoc = other.yLoc;
        wDim = other.wDim;
        hDim = other.hDim;
        imagePath = other.imagePath;
    }

    VisualComponent& VisualComponent::operator=(VisualComponent&& other) noexcept {
        if (instanceID)
            sprites.at(imagePath).removeElement(instanceID);

        instanceID = other.instanceID;
        other.instanceID = NULL;
        xLoc = other.xLoc;
        yLoc = other.yLoc;
        wDim = other.wDim;
        hDim = other.hDim;
        imagePath = other.imagePath;

        return *this;
    }

    void update() {
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(shaderProgram);

        for (auto& sprite : sprites) {
            sprite.second.draw();
        }

        glUseProgram(NULL); // Could rebind previous bound program

        SwapBuffers(deviceContext);
    }

    // Copied from https://www.khronos.org/opengl/wiki/Load_OpenGL_Functions
    void* GetAnyGLFuncAddress(const char* name)
    {
        void* p = (void*)wglGetProcAddress(name);
        if (p == 0 ||
            (p == (void*)0x1) || (p == (void*)0x2) || (p == (void*)0x3) ||
            (p == (void*)-1))
        {
            HMODULE module = LoadLibraryA("opengl32.dll");
            p = (void*)GetProcAddress(module, name);
        }

        return p;
    }

    void init(HWND windowHandle, GLint x, GLint y, GLsizei windowWidth, GLsizei windowHeight) {
        // Create OpenGL context
        deviceContext = GetDC(windowHandle);
        {
            // Copied from https://www.khronos.org/opengl/wiki/Creating_an_OpenGL_Context_(WGL)
            PIXELFORMATDESCRIPTOR pfd =
            {
                sizeof(PIXELFORMATDESCRIPTOR),
                1,
                PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,    // Flags
                PFD_TYPE_RGBA,        // The kind of framebuffer. RGBA or palette.
                32,                   // Colordepth of the framebuffer.
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                24,                   // Number of bits for the depthbuffer
                8,                    // Number of bits for the stencilbuffer
                0,                    // Number of Aux buffers in the framebuffer.
                PFD_MAIN_PLANE,
                0, 0, 0, 0
            };
            int pixelFormat = ChoosePixelFormat(deviceContext, &pfd);
            assert(pixelFormat != 0);
            SetPixelFormat(deviceContext, pixelFormat, &pfd);
        }
        context = wglCreateContext(deviceContext);
        wglMakeCurrent(deviceContext, context);

        // Initialise opengl functions with Glad
        {
            int temp = gladLoadGLLoader((GLADloadproc)GetAnyGLFuncAddress);
            assert(temp != 0);
        }

        // Set viewport for game, will not change size
        glViewport(x, y, windowWidth, windowHeight);

        // Create shader program
        {
            const char* vertexShaderSource = "#version 450 core\n"
                "layout (location = 0) in vec2 aPos;\n"
                "layout (location = 1) in vec2 aDim;\n"
                "out VS_OUT {\n"
                "   vec2 dim;\n"
                "} vs_out;\n"
                "void main()\n"
                "{\n"
                "   gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);\n"
                "   vs_out.dim = aDim;\n"
                "}\0";
            GLuint vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
            glShaderSource(vertexShaderID, 1, &vertexShaderSource, NULL);
            glCompileShader(vertexShaderID);

            const char* geomentryShaderSource = "#version 450 core\n"
                "layout (points) in;\n"
                "layout (triangle_strip, max_vertices = 4) out;\n"
                "in VS_OUT {\n"
                "   vec2 dim;\n"
                "} gs_in[];\n"
                "out vec2 texCoord;\n"
                "void main()\n"
                "{\n"
                "   gl_Position = gl_in[0].gl_Position;\n"
                "   texCoord = vec2(0.0, 1.0);\n"
                "   EmitVertex();\n"
                "   gl_Position = gl_in[0].gl_Position + vec4(gs_in[0].dim.x, 0.0, 0.0, 0.0);\n"
                "   texCoord = vec2(1.0, 1.0);\n"
                "   EmitVertex();\n"
                "   gl_Position = gl_in[0].gl_Position + vec4(0.0, -gs_in[0].dim.y, 0.0, 0.0);\n"
                "   texCoord = vec2(0.0, 0.0);\n"
                "   EmitVertex();\n"
                "   gl_Position = gl_in[0].gl_Position + vec4(gs_in[0].dim.x, -gs_in[0].dim.y, 0.0, 0.0);\n"
                "   texCoord = vec2(1.0, 0.0);\n"
                "   EmitVertex();\n"
                "   EndPrimitive();\n"
                "}\0";
            GLuint geomentryShaderID = glCreateShader(GL_GEOMETRY_SHADER);
            glShaderSource(geomentryShaderID, 1, &geomentryShaderSource, NULL);
            glCompileShader(geomentryShaderID);

            const char* fragmentShaderSource = "#version 450 core\n"
                "out vec4 FragColor;\n"
                "in vec2 texCoord;\n"
                "uniform sampler2D ourTexture;\n"
                "void main()\n"
                "{\n"
                "    FragColor = texture(ourTexture, texCoord);\n"
                "}\n";
            GLuint fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
            glShaderSource(fragmentShaderID, 1, &fragmentShaderSource, NULL);
            glCompileShader(fragmentShaderID);

            shaderProgram = glCreateProgram();
            glAttachShader(shaderProgram, vertexShaderID);
            glAttachShader(shaderProgram, geomentryShaderID);
            glAttachShader(shaderProgram, fragmentShaderID);
            glLinkProgram(shaderProgram);
        }

        stbi_set_flip_vertically_on_load(true); // Need to do this before loading any images
    }

    void cleanup() {
        // Need to empty the 'sprites' container for the Sprite destructors before deleting openGL context
        sprites.clear(); // VisualComponents should to be destroyed before this
        wglMakeCurrent(deviceContext, NULL);
        wglDeleteContext(context);
    }
}