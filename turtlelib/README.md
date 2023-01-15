# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
     1. Include it as an independent function
     2. Put it in a class or struct as a member function
     3. Create a normalize class

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
     1. This is easy to implement. Moreover, the function can be seen as part of the useful interface to the Vector2D struct even though it does not need direct access to the representation of the struct. This is according to C.5. However, it cannot directly access the private members of a class.
     2. According to C.4, a member function should be made only if it needs direct access to the representation of a class. This member function will need to use other member functions in the class and access private members in order to have the normalize functionality. One drawback is that it can only be called on an object of the class it is defined in.
     3. Using a class can protect data from unintended modification. However, according to C.1, classes should be used for ease of comprehension. Creating a class based on one functionality is too complicated.

   - Which of the methods would you implement and why?
     I implemented option #1 because there was no need for the function to use member variables of a class. It just required one Vector2D argument, performed the necessary operations, and returned a normalized vector.

2. What is the difference between a class and a struct in C++?
   The difference between a class and a struct is the default access level of the members. A class has private members by default while a struct has public members by default.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
   Vector2D is a struct rather than a class because the elements in the vector can vary independently. Transform2D is a class rather than a struct because it has interdependent elements. Changing an element in a transformation matrix can affect the entire matrix. This is according to C.2.

   Another reason is that Vector2D has functions that do not need direct access to the representation of a class while Transform2D has member functions that need direct access to the private members of the class. This is according to C.4.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
   Some of the constructors in Transform2D are explicit because they are single-argument constructors. Single-argument constructors can be used in implicit type conversions, which can lead to unexpected results. Therefore, they should be declared as explicit to avoid unintended conversions. This is according to C.46.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
   Transform2D::inv() is declared const because it returns a new Transform2D object instead of modifying the object itself. Transform2D::operator*=() is not declared const because it is changing the private members of Transform2D. This is according to Con.1, which states to make objects immutable by default and make them non-const only when there is a need to change their values.