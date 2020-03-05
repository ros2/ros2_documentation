

This document is a declaration of software quality for the `boost` ROS external dependency, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `Boost` Quality Declaration

The ROS external dependency `Boost` claims to be in the **Quality Level 1** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [External dependencies requirements for Quality Level 1 in REP-2XXX](https://www.ros.org/reps/rep-2004.html).

External dependencies are evaluated on a case by case basis, some items may not adhere to standards defined for ROS packages.

## Version Policy

### Version Scheme

It is not publicly available the information regarding `boost` uses a versioning system to manage automatically their version schemes. However, it is explicitly stated in their webpage [FAQ](https://www.boost.org/users/faq.html):

*"The scheme is x.y.z, where x is incremented only for massive changes, such as a reorganization of many libraries, y is incremented whenever a new library is added, and z is incremented for maintenance releases. y and z are reset to 0 if the value to the left changes."*

### API Stability // ABI Stability

`Boost` libraries do not make any guarantee regarding API or ABI stability within their versions schemes. However, from their [requirements guidelines](https://www.boost.org/development/requirements.html) documentation, they encourage authors of the libraries not to introduce breaking changes without notifications a few releases before the change is published.

### Public API Declaration

Although there is no specific documentation on the Public API requirements, it is available a [Header Policy](https://www.boost.org/development/header.html) with guidelines for code reusability, implicitly suggesting that code available in the library headers is the preferred way to interface with the libraries.

## Change Control Process

There is no specific implementation/guidelines stated as to how authors should maintain their libraries. 

Libraries without a maintainer will be maintained by the [Community Maintenance Team](https://svn.boost.org/trac10/wiki/CommunityMaintenance), and it is stated a clear process of how changes will be addressed, using a "propose and review". This suggests the implicit existence of this process for the main Boost libraries.

## Documentation

`Boost` provides documentation guidelines preferred for their libraries. Although there is not strict control over how the libraries are to be maintained neither strict standards, several topics to be addressed [are suggested](https://www.boost.org/development/requirements.html#Documentation) and there is a standard documentation page with a [defined structure](https://www.boost.org/doc/libs/1_72_0/more/writingdoc/structure.html) available to ease providing documentation.

Feature and API documentation are not explicitly addressed, but the available template provides general descriptions about how the elements of the libraries should be documented.

### License and Copyright Statements

The license used for `Boost` is The *Boost Software License*. And this is the template provided for source and header files:

    //          Copyright _Joe Coder 2004 - 2006_.
    // Distributed under the Boost Software License, Version 1.0.
    //    (See accompanying file LICENSE_1_0.txt or copy at
    //          https://www.boost.org/LICENSE_1_0.txt)

It is not stated if the library files licenses/copyright notices are checked with automated tests.

## Testing

`Boost` includes [guidelines](https://www.boost.org/development/test.html) regarding test policies. The guidelines declare protocols for testing each new feature, and requirements to be able to run automated regression tests. This should cover new features, and surely the main public API.

There is no information available publicly regarding coverage or performance testing at all, neither the use of any linters or static analysis in particular.

## Dependencies

`Boost` libraries encourage libraries developers and maintainer to not use external dependencies to Boost or the C++ Standard Library at all. It's recommended not to use other Boost libraries as well unless [the benefits outweigh the costs](https://www.boost.org/development/reuse.html).

## Platform Support

There is no particular information regardless Boost should provide support for any particular platform. Portability is encouraged, part of the [Portability Requirements](https://www.boost.org/development/requirements.html) states that the libraries should not depend on any particular platform or compiler and it's asked that all libraries provide implementations for at least two common operative systems.

## Conclusion summary

Boost libraries do not match the complete list of requirements established for ROS internal dependencies, their main weak point is the complete absence of performance testing and coverage. Also, there is no information available stating if code is tested against specific linters or other static analysis tools.

 However, considering the availability of guidelines for most of the development states in their software life-cycle, their involvement with the open-source community, and the fact that the Boost code is used in minimal parts of the ROS code, it was decided by the XYZ committee that Boost libraries will be considered to be of quality level 1 on March 2020.