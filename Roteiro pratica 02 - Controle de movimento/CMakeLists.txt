cmake_minimum_required(VERSION 3.8)

# Nome do pacote
project(pratica_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontra o sistema de construção do ROS 2
find_package(ament_cmake REQUIRED)

# Configura os testes e verificações do pacote
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Desativa a verificação de copyright
  set(ament_cmake_copyright_FOUND TRUE)
  
  # Desativa a verificação do cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  
  # Encontra as dependências necessárias para os testes
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
