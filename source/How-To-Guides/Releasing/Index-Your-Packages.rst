Index Your Packages
===================

Are you releasing a new ROS package into a ROS distribution?
Make the process faster by indexing your packages first.

Put your ROS packages into a public repository
----------------------------------------------

If you haven't done so already, put the source code of your ROS packages into a public git repository.
All packages released into ROS must be open source.
You can host code anywhere, but GitHub is recommended because it gives you the option to enable pull request jobs.
Here are some choices:

* `GitHub <https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-new-repository>`__ **Recommended**
* `GitLab <https://docs.gitlab.com/ee/user/project/repository/>`__
* `Bitbucket <https://support.atlassian.com/bitbucket-cloud/docs/create-a-git-repository/>`__

Give your packages an OSI Approved license
------------------------------------------
Choose an `OSI approved license <https://opensource.org/licenses>`__ and give it to your ROS packages.
If you're having trouble deciding, consider using the license used by most of the core ROS 2 packages: `Apache-2.0 license <https://opensource.org/license/apache-2-0>`__.

For each ``package.xml`` in your repository, put the SPDX short identifier of the license in the ``<license>`` tag in your ``package.xml``.

If all of your ROS packages have the same license, or if there's only one ROS package in your repository, create a file called ``LICENSE`` at the root of your repository and put the text of the license you chose in it.
If the ROS packages in your repository have different licenses, create a ``LICENSE`` file adjacent to every ``package.xml`` file.

Give your packages REP 144 compliant names
------------------------------------------
Packages released into a ROS distribution must have names that comply with `REP 144 <https://www.ros.org/reps/rep-0144.html>`__.
Read the full REP to understand the rules.
If one of your ROS package names doesn't comply, then change the name before continuing.

Decide what ROS distribution you want to release into
-----------------------------------------------------
Decide what ROS distribution you want to release your packages into.
At a minimum, you should release your packages into `ROS Rolling <https://docs.ros.org/en/rolling>`__ so that your ROS packages are automatically included in the next ROS release.
You may also want to release into any active ROS distributions, but this is up to you.

Create a GitHub account
-----------------------
`Create a GitHub account <https://docs.github.com/en/get-started/start-your-journey/creating-an-account-on-github>`__ if you don't already have one.
You don't have to host the source code of your ROS packages on GitHub, but you will need an account to index and release packages.

Fork and clone ros/rosdistro
----------------------------
`Fork <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo>`__ the `ros/rosdistro <https://github.com/ros/rosdistro/>`__ repository.
You only need to do this step once on your account.
The fork will be used every time you do a release.

Make changes to your fork
-------------------------
Remember the ROS distributions you decided to release into?
Each ROS distribution has a folder in the `ros/rosdistro <https://github.com/ros/rosdistro/>`__ repository.
For example, the name of the ROS Rolling folder is ``rolling``.
For each ROS distribution you want to release into:

1. fill out the following template
2. put the filled-out template into the ``distribution.yaml`` file in the corresponding ROS distribution's folder

.. code-block:: yaml

  YOUR-REPO-NAME:
    source:
      type: git
      url: https://YOUR-GIT-REPO-URL.git
      version: YOUR-BRANCH-NAME
    status: YOUR-STATUS

Here's how to fill out each item:

* YOUR-REPO-NAME: This is an arbitrary human-readable name.
  For repos hosted on GitHub, use the lowercase name of your repository not including the organization.
  For example, the repository name of ``https://github.com/ros2/rosidl`` is ``rosidl``.
* YOUR-GIT-REPO-URL: This is the https URL from which one could ``git clone`` your repository
  For example, the git repo URL of ``https://github.com/ros2/rosidl`` is ``https://github.com/ros2/rosidl.git``.
  It is important that this URL ends in ``.git``, or it will fail to pass the linters.
* YOUR-BRANCH-NAME: This is the git branch on your repository from which you will release your package into this ROS distribution.
  This is commonly one of: ``main``, ``master``, or the name of the ROS distribution itself.
  For example, the `rosidl repository <https://github.com/ros2/rosidl>`__ uses the branch ``rolling`` to hold changes to be released into ROS Rolling.
* YOUR-STATUS: This is a status from the list in `REP 141 <https://www.ros.org/reps/rep-0141.html#distribution-file>`__.
  You likely want either ``maintained`` or ``developed``.

Open a pull request to ros/rosdistro
------------------------------------
`Open a pull request <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request>`__ to `ros/rosdistro <https://github.com/ros/rosdistro/>`__ with the branch that you made your changes to.
Wait a few days for it to be reviewed.

What happens next
-----------------
You've now done everything required to index your ROS packages.
One of the reviewers will look at your pull request and decide if it `satisfies the review guidelines <https://github.com/ros/rosdistro/blob/master/REVIEW_GUIDELINES.md>`__.
The reviewer may either approve your changes as is, or give you actionable feedback.
Once the pull request meets the review guidelines it will be merged, and your packages will appear on the `ROS Index <https://index.ros.org/>`__.

You've completed an important step toward releasing your package.
Proceed to the next guide: :doc:`First Time Release <First-Time-Release>`.
