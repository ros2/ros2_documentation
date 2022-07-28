.. warning::

   If the file ``~/.config/bloom`` exists on your computer, it is likely that you have done this before so you should skip this section.

During the release process, multiple HTTPS Git operations will be performed that require password authentication.
To avoid being repeatedly asked for a password, a `Personal Access Token (PAT) <https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token>`_ will be set up.

Create a Personal Access Token by:

#. Log in to GitHub and go to `Personal access tokens <https://github.com/settings/tokens>`_.
#. Click the **Generate new token** button.
#. Set **Note** to something like ``Bloom token``.
#. Set **Expiration** to **No expiration**.
#. Tick the ``public_repo`` and ``workflow`` checkboxes.
#. Click the **Generate token** button.

After you have created the token, you will end up back at the *Personal access tokens* page.
**Copy the alphanumeric token** that is highlighted in green.

Save your GitHub username and PAT to a new file called ``~/.config/bloom``, with the format below:

.. code-block:: text

   {
      "github_user": "<your-github-username>",
      "oauth_token": "<token-you-created-for-bloom>"
   }
