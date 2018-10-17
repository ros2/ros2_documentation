
New features in ROS 2 interfaces
================================

**INCOMPLETE**

The `ROS 2 interface definition language <About-ROS-Interfaces>`_\ , or IDL, is closely related to the `ROS 1 IDL <http://wiki.ros.org/msg>`__.
Most existing ROS 1 ``.msg`` and ``.srv`` files should be usable as-is with ROS 2.
Atop that existing feature set, the ROS 2 IDL introduces some new features, namely:


* **bounded arrays**\ : Whereas the ROS 1 IDL allows unbounded arrays (e.g., ``int32[] foo``\ ) and fixed-size arrays (e.g., ``int32[5] bar``\ ), the ROS 2 IDL further allows bounded arrays (e.g., ``int32[<=5] bat``\ ).
  There are use cases in which it's important to be able to place an upper bound on the size of an array without committing to always using that much space (e.g., in a real-time system in which you need to preallocate all memory that will be used during execution).
* **bounded strings**\ : Whereas the ROS 1 IDL allows unbounded strings (e.g., ``string foo``\ ), the ROS 2 IDL further allows bounded strings (e.g., ``string<=5 bar``\ ).
* **default values**\ : Whereas the ROS 1 IDL allows constant fields (e.g., ``int32 X=123``\ ), the ROS 2 IDL further allows default values to be specified (e.g., ``int32 X 123``\ ).
  The default value is used when constructing a message/service object and can be subsequently overridden by assigning to the field.
  *Note: in ROS Ardent, default values are not supported for complex types or string arrays or strings with encoding.*\ Note: in ROS Bouncy, default values are not supported for complex types or string with encoding.
