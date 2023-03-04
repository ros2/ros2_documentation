.. _TestingMain:

Testing
=======

Why automatic tests?
--------------------

Here are some of the many good reasons why should we have automated tests:

* You can make incremental updates to your code more quickly. ROS has hundreds of packages with many interdependencies, so it can be hard to anticipate the problems a small change might cause. If your change passes the unit tests, you can be more confident that you haven't introduced problems — or at least the problems aren't your fault.
* You can refactor your code with greater confidence. Passing the unit tests verifies that you haven't introduced any bugs while refactoring. This gives you this wonderful freedom from change fear!
* It leads to better designed code. Unit tests force you to write your code so that it can be more easily tested. This often means keeping your underlying functions and framework separate, which is one of our design goals with ROS code.
* They prevent recurring bugs (bug regressions). It's a good practice to write a unit test for every bug you fix. In fact, write the unit test before you fix the bug. This will help you to precisely, or even deterministically, reproduce the bug, and much more precisely understand what the problem is. As a result, you will also create a better patch, which you can then test with your regression test to verify that the bug is fixed. That way the bug won't accidentally get reintroduced if the code gets modified later on. It also means that it will be easier to convince the reviewer of the patch that the problem is solved, and the contribution is of high quality.
* Other people can work on your code more easily (an automatic form of documentation). It can be hard to figure out whether or not you've broken someone else's code when you make a change. The unit tests are a tool for other developers to validate their changes. Automatic tests document your coding decisions, and communicate to other developers automatically about their violation. Thus tests become documentation for your code — a documentation that does not need to be read for the most time, and when it does need to be inspected the test system will precisely indicate what to read (which tests fail). By writing automatic tests you make other contributors faster. This improves the entire ROS project.
* It is much easier to become a contributor to ROS if we have automated unit tests. It is very difficult for new external developers to contribute to your components. When they make changes to code, they are often doing it in the blind, driven by a lot of guesswork. By providing a harness of automated tests, you help them in the task. They get immediate feedback for their changes. It becomes easier to contribute to a project, and new contributors to join more easily. Also their first contributions are of higher quality, which decreases the workload on maintainers. A win-win!
* Automatic tests simplify maintainership. Especially for mature packages, which change more slowly, and mostly need to be updated to new dependencies, an automatic test suite helps to very quickly establish whether the package still works. This makes it much easier to decide whether the package is still supported or not.
* Automatic tests amplify the value of Continuous Integration. Regression tests, along with normal scenario-based requirements tests, contribute to overall body of automated tests for your component. Your component is better tested against evolution of other APIs that it depends on (CI servers will tell you better and more precisely what problems develop in your code).

Perhaps the most important benefit of writing tests is that tests make you a good citizen.
Tests influence quality in the long term.
It is a well accepted practice in many open-source projects.
By writing regressions tests, you are contributing to long term quality of the ROS ecosystem.

Is this all coming for free?
----------------------------

Of course, there is never free lunch.
To get the benefits of testing, some investment is necessary.

* You need to develop a test, which sometimes may be difficult or costly. Sometimes it might also be nontrivial, as the test should be automatic. Things get particularly hairy if your tests should involve special hardware (they should not: try to use simulation, mock the hardware, or narrow down the test to a smaller software problem) or require external environment, for instance human operators.
* Regression tests and other automatic tests need to be maintained. When the design of the component changes, a lot of tests become invalidated (for instance they no longer compile, or throw runtime exceptions related to the API design). These tests fail not only because the redesign re-introduced bugs but also because they need to be updated to the new design. Occasionally, with bigger redesigns, old regression tests should be dropped.
* Large bodies of tests can take a long time to run, which can increase Continuous Integration server costs.

Available Tutorials:
--------------------

.. toctree::
   :maxdepth: 1

   CLI
   Cpp
   Python
