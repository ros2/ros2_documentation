Github Manual Authorization
===========================

.. contents:: Table of Contents
   :depth: 3
   :local:

If you have two-factor authentication enabled for github, bloom cannot log in with your github
password alone. There are two steps:

#. Create a personal access token on github and tell bloom about it.
   This is used for pull-requestopening.
#. Re-route https traffic through ssh (assuming you have public-key authentication enabled).
   (This will only enable git push and pull, but not pull-request opening)

Token-based Authentication
--------------------------

Create an access token
^^^^^^^^^^^^^^^^^^^^^^

Go to the github website and login. Open the settings page, go to `Developer Settings <https://github.com/settings/developers>`_
and click on `Personal access tokens <https://github.com/settings/tokens>`_ in the menu on the left.

Click the "Generate new token" button, and follow the instructions.
Make sure that the "public_repo" and "workflow" authorization is granted under the repo subtree.
Set the description to something like "Bloom token".
After you have created the token, you will end up back at the "Personal access tokens" page.
The new token will be highlighted in green. Copy the alphanumeric token to the clipboard.

Configure bloom to use the token
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open ``~/.config/bloom`` in a text editor. If it doesn't exist, create it.
Enter the following text into the file:

.. code-block:: text

   {
      "github_user": "<your-github-username>",
      "oauth_token": "<token-you-created-for-bloom>"
   }

Filling in your username and the token you made in the previous step. Save the file.
From now on, bloom should be able to automatically create pull requests to rosdistro for
you when you do releases.

You may still see request for username/password from Bloom when releasing.
If so, the next step is to redirect the https requests to use your ssh authentication for
the repository pull and push.

Re-route https through ssh
--------------------------

As suggested `here <http://answers.ros.org/question/234494/diagnosing-issues-with-bloom-github-two-factor-authentication/>`_,
you can also re-route bloom's https traffic through ssh by editing your ``.gitconfig`` file:

.. code-block:: text

   # Always use ssh for github, even if the remote URL uses https or git
   [url "git@github.com:"]
     insteadOf = git://github.com/
   [url "git@github.com:"]
     insteadOf = https://github.com/

If you choose this option, be sure not to forget
`enable ssh connection with github <https://help.github.com/articles/generating-an-ssh-key/>`_.

Bash Script for setting up OAuth Token
--------------------------------------

In case you use automatic setup scripts for your computer, you might find this script helpful:

.. code-block:: bash

   # Install Bloom Github oauth access token for releasing ROS packages
   function installros_github_token() {
   read -p "Your Github username: " userName
   read -p "A Github oauth token for your account: " oAuthToken
   rm -f $HOME/.config/bloom
   cat <<EOF >> $HOME/.config/bloom
   {
      "github_user": "$userName",
      "oauth_token": "$oAuthToken"
   }
   EOF
   }