cmake_minimum_required(VERSION 3.10)
project(Boids)

set(CMAKE_CXX_STANDARD 17)

# 1. Configurações Específicas para macOS (Apple)
if(APPLE)
    # Suprime avisos de que o OpenGL é obsoleto no macOS
    add_compile_options(-Wno-deprecated)

    # Encontra os Frameworks nativos do sistema
    find_library(OPENGL_LIB OpenGL)
    find_library(GLUT_LIB GLUT)

    # Define as bibliotecas a serem linkadas
    set(LIBS ${OPENGL_LIB} ${GLUT_LIB})
else()
    # Fallback para Linux/Windows (caso você mude de ideia)
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    set(LIBS ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
endif()

# 2. Define o executável
add_executable(boids main.cpp)

# 3. Linka as bibliotecas encontradas ao executável
target_link_libraries(boids ${LIBS})
