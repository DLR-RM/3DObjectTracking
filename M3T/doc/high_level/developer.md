\page developer Developer Guide

On this page, useful tips and information for developers are collected.

#### Code Style

A Clang style file `.clang-format` is used to enforce a uniform code style and formatting.  
It is based on the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) which should be considered for any style decision.

Some style conventions are listed below:

* Getters/setters are in snake_case
* Main functions are CamelCase
* Class parameters end with an underscore `_`
* Direct initialization of objects with `MyClass my_class{}`
* The order of the classes and functions should be identical in the `.h` header and `.cpp` source files
* Variable names describing pointers end with `_ptr` or `_ptrs` for multiple pointers.

Best practices:

* Avoid deeply nested if-clauses
* Public functions are declared before private functions in the header file
* Avoid abbreviations like `img` or unhelpful variable names like `var`
* Operations in the constructor and destructor should not throw any exceptions

#### Setup Functionalities of classes

Every class has a parameter `set_up_` and a function `SetUp()`. The `SetUp` function first checks if referenced objects are already set up correctly. It then checks if the class' parameters are set and valid and performs a set up. After `SetUp()` was called and executed successfully, the parameter `set_up_` is set to true. Typically, only the tracker object should call the `SetUp()` functions of its associated classes.

#### Comment Conventions

* Every class should have a `\brief` description
* Comments always start with a capital letter
* Doxygen keywords like `@param`, `@return` , `\brief`, ... can be used in class and function descriptions