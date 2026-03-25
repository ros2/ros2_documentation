Add Your Project
================

Use the form below to generate the YAML entry for your organization or project.
Once generated, you can copy the YAML snippet and submit a pull request to the
`adopters.yaml <https://github.com/ros2/ros2_documentation/blob/rolling/source/The-ROS2-Project/Adopters/adopters.yaml>`__
file on the ``rolling`` branch.

Policy
------

This list is **self-reported and self-attested**.
Entries are accepted with minimal scrutiny unless a complaint is received.
Since contributions come via Pull Request, they are easy to audit and can be cleaned up later if necessary.

How to contribute
-----------------

1. Fill in the form below.
2. Click **Generate YAML** to produce the snippet.
3. Click **Open PR on GitHub** to open the file in GitHub's web editor (the YAML is copied to your clipboard automatically).
4. Paste the generated YAML at the end of the ``adopters:`` list in the file.
5. Commit the change and open a pull request.

.. note::

   All pull requests to the ROS 2 documentation repository require a
   `Developer Certificate of Origin (DCO) <https://developercertificate.org/>`__ sign-off.
   If you use the GitHub web editor, the
   `DCO bot <https://github.com/apps/dco>`__ will comment on your PR with
   instructions to add the sign-off if it is missing.
   To sign off via the command line, use ``git commit --signoff``.

.. raw:: html

   <div class="adopters-form-container">
   <form id="adopters-yaml-form">

     <div class="form-group">
       <label for="field-organization">Organization *</label>
       <span class="form-hint">Company or institution name</span>
       <input type="text" id="field-organization" placeholder="e.g., Acme Robotics Inc.">
     </div>

     <div class="form-group">
       <label for="field-organization-url">Organization URL</label>
       <span class="form-hint">Optional</span>
       <input type="url" id="field-organization-url" placeholder="https://www.example.com">
     </div>

     <div class="form-group">
       <label for="field-project">Project *</label>
       <span class="form-hint">The specific project using ROS</span>
       <input type="text" id="field-project" placeholder="e.g., Autonomous Forklift">
     </div>

     <div class="form-group">
       <label for="field-project-url">Project URL</label>
       <span class="form-hint">Optional</span>
       <input type="url" id="field-project-url" placeholder="https://www.example.com/project">
     </div>

     <div class="form-group">
       <label>Domain * <span class="form-hint">(select one or more)</span></label>
       <div class="domain-checkboxes">
         <label><input type="checkbox" name="domain" value="Agriculture"> Agriculture</label>
         <label><input type="checkbox" name="domain" value="Aerial/Drone"> Aerial/Drone</label>
         <label><input type="checkbox" name="domain" value="Automotive"> Automotive</label>
         <label><input type="checkbox" name="domain" value="Components"> Components</label>
         <label><input type="checkbox" name="domain" value="Construction"> Construction</label>
         <label><input type="checkbox" name="domain" value="Consumer Robot"> Consumer Robot</label>
         <label><input type="checkbox" name="domain" value="Defense/Government"> Defense/Government</label>
         <label><input type="checkbox" name="domain" value="Education"> Education</label>
         <label><input type="checkbox" name="domain" value="Energy"> Energy</label>
         <label><input type="checkbox" name="domain" value="Healthcare/Medical"> Healthcare/Medical</label>
         <label><input type="checkbox" name="domain" value="Humanoid"> Humanoid</label>
         <label><input type="checkbox" name="domain" value="Logistics/Warehouse"> Logistics/Warehouse</label>
         <label><input type="checkbox" name="domain" value="Manufacturing"> Manufacturing</label>
         <label><input type="checkbox" name="domain" value="Marine"> Marine</label>
         <label><input type="checkbox" name="domain" value="Research"> Research</label>
         <label><input type="checkbox" name="domain" value="Space"> Space</label>
         <label><input type="checkbox" name="domain" value="Service Robot"> Service Robot</label>
       </div>
     </div>

     <div class="form-group">
       <label for="field-date-added">Date Added *</label>
       <span class="form-hint">Auto-generated (YYYY-MM-DD)</span>
       <input type="text" id="field-date-added" readonly style="width: 120px; background: #e9ecef;">
     </div>

     <div class="form-group">
       <label for="field-country">Country *</label>
       <span class="form-hint">Select one or more countries</span>
       <div style="display: flex; gap: 0.5rem; align-items: center; flex-wrap: wrap;">
         <select id="field-country" style="width: 280px;">
           <option value="">-- Select a country --</option>
         </select>
         <button type="button" id="adopters-add-country-btn" class="btn btn-secondary" style="margin-top: 0;">Add</button>
       </div>
       <div id="adopters-selected-countries" class="adopters-country-tags"></div>
     </div>

     <div class="form-group">
       <label for="field-description">Description *</label>
       <span class="form-hint">Brief explanation of how you use ROS</span>
       <textarea id="field-description" placeholder="e.g., Autonomous navigation for warehouse logistics using ROS 2 and Nav2."></textarea>
     </div>

     <div id="adopters-form-errors" style="display: none;"></div>

     <button type="button" id="adopters-generate-btn" class="btn btn-primary">Generate YAML</button>
     <button type="button" id="adopters-copy-btn" class="btn btn-secondary" style="display: none;">Copy to Clipboard</button>
     <button type="button" id="adopters-open-pr-btn" class="btn btn-success" style="display: none;">Open PR on GitHub</button>

     <pre id="adopters-yaml-output" style="display: none;"></pre>

   </form>
   </div>
