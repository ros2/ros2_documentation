/**
 * ROS 2 Adopters - Table filtering and YAML generator.
 *
 * Copyright 2026 Sony Group Corporation.
 * Licensed under the Apache License, Version 2.0.
 *
 * NOTE: The VALID_DOMAINS array below must be kept in sync
 * with the corresponding constants in plugins/adopters_schema.py.
 * Any additions or changes must be applied to both files.
 */

/* ===== Table filtering ===== */

function initAdoptersTableFilters() {
  var filterDomain = document.getElementById('adopters-filter-domain');
  var filterCountry = document.getElementById('adopters-filter-country');
  var filterSearch = document.getElementById('adopters-filter-search');
  var showAllToggle = document.getElementById('adopters-show-all');

  if (!filterDomain) return;  // Not on the adopters table page.

  // Compute the cutoff date (3 years ago) as YYYY-MM string.
  var now = new Date();
  var cutoffYear = now.getFullYear() - 3;
  var cutoffMonth = String(now.getMonth() + 1).padStart(2, '0');
  var cutoffDate = cutoffYear + '-' + cutoffMonth;

  function applyFilters() {
    var domain = filterDomain.value;
    var country = filterCountry.value;
    var search = filterSearch.value.toLowerCase();
    var showAll = showAllToggle.checked;

    var rows = document.querySelectorAll('.adopters-table tbody tr');
    rows.forEach(function(row) {
      var domainMatch = !domain || row.getAttribute('data-domains').indexOf(domain) !== -1;
      var countryMatch = !country || row.getAttribute('data-countries').indexOf(country) !== -1;
      var searchMatch = !search || row.textContent.toLowerCase().indexOf(search) !== -1;

      // Date-based filtering: hide entries older than 3 years unless toggle is on.
      var dateAdded = row.getAttribute('data-date-added') || '';
      var withinWindow = showAll || dateAdded >= cutoffDate;

      row.style.display = (domainMatch && countryMatch && searchMatch && withinWindow) ? '' : 'none';
    });
  }

  filterDomain.addEventListener('change', applyFilters);
  filterCountry.addEventListener('change', applyFilters);
  filterSearch.addEventListener('input', applyFilters);
  showAllToggle.addEventListener('change', applyFilters);

  // Apply initial filter to hide old entries by default.
  applyFilters();
}

/* ===== YAML Generator Form ===== */

function initAdoptersForm() {
  var form = document.getElementById('adopters-yaml-form');
  if (!form) return;  // Not on the form page.

  var generateBtn = document.getElementById('adopters-generate-btn');
  var openPrBtn = document.getElementById('adopters-open-pr-btn');
  var copyBtn = document.getElementById('adopters-copy-btn');
  var output = document.getElementById('adopters-yaml-output');
  var errorDiv = document.getElementById('adopters-form-errors');

  // Auto-populate date_added with current YYYY-MM.
  var dateField = document.getElementById('field-date-added');
  if (dateField) {
    var now = new Date();
    var yyyy = now.getFullYear();
    var mm = String(now.getMonth() + 1).padStart(2, '0');
    var dd = String(now.getDate()).padStart(2, '0');
    dateField.value = yyyy + '-' + mm + '-' + dd;
  }

  function getFormValues() {
    var domains = [];
    var checkboxes = form.querySelectorAll('input[name="domain"]:checked');
    checkboxes.forEach(function(cb) { domains.push(cb.value); });

    // Parse comma-separated country codes.
    var countryRaw = form.querySelector('#field-country').value.trim().toUpperCase();
    var countries = countryRaw.split(/[,\s]+/).filter(function(c) { return c.length > 0; });

    return {
      organization: form.querySelector('#field-organization').value.trim(),
      organization_url: form.querySelector('#field-organization-url').value.trim(),
      project: form.querySelector('#field-project').value.trim(),
      project_url: form.querySelector('#field-project-url').value.trim(),
      domain: domains,
      date_added: form.querySelector('#field-date-added').value.trim(),
      country: countries,
      description: form.querySelector('#field-description').value.trim()
    };
  }

  function validateForm(values) {
    var errors = [];
    if (!values.organization) errors.push('Organization is required.');
    if (!values.project) errors.push('Project is required.');
    if (values.domain.length === 0) errors.push('At least one domain must be selected.');
    if (!values.date_added) errors.push('Date added is required.');
    if (values.date_added && !/^\d{4}-(0[1-9]|1[0-2])-(0[1-9]|[12]\d|3[01])$/.test(values.date_added)) {
      errors.push('Date added must be in YYYY-MM-DD format.');
    }
    if (values.country.length === 0) errors.push('At least one country is required.');
    values.country.forEach(function(c) {
      if (c.length !== 2 || !/^[A-Z]{2}$/.test(c)) {
        errors.push('Country "' + c + '" must be a 2-letter ISO 3166-1 alpha-2 code (e.g., US, JP, DE).');
      }
    });
    if (!values.description) errors.push('Description is required.');
    if (values.organization_url && !isValidUrl(values.organization_url)) {
      errors.push('Organization URL is not a valid URL.');
    }
    if (values.project_url && !isValidUrl(values.project_url)) {
      errors.push('Project URL is not a valid URL.');
    }
    return errors;
  }

  function isValidUrl(str) {
    try { new URL(str); return true; } catch (e) { return false; }
  }

  function yamlEscape(str) {
    // Quote strings that contain YAML-special characters.
    if (/[:#\[\]{}&*!|>'"%@`,?]/.test(str) || str !== str.trim()) {
      return '"' + str.replace(/\\/g, '\\\\').replace(/"/g, '\\"') + '"';
    }
    return '"' + str + '"';
  }

  function generateYaml(values) {
    var lines = [];
    lines.push('  - organization: ' + yamlEscape(values.organization));
    if (values.organization_url) {
      lines.push('    organization_url: ' + yamlEscape(values.organization_url));
    }
    lines.push('    project: ' + yamlEscape(values.project));
    if (values.project_url) {
      lines.push('    project_url: ' + yamlEscape(values.project_url));
    }
    lines.push('    domain:');
    values.domain.forEach(function(d) {
      lines.push('      - ' + d);
    });
    lines.push('    date_added: ' + yamlEscape(values.date_added));
    lines.push('    country:');
    values.country.forEach(function(c) {
      lines.push('      - ' + c);
    });
    lines.push('    description: ' + yamlEscape(values.description));
    return lines.join('\n');
  }

  generateBtn.addEventListener('click', function(e) {
    e.preventDefault();
    var values = getFormValues();
    var errors = validateForm(values);

    if (errors.length > 0) {
      errorDiv.innerHTML = '<ul>' + errors.map(function(e) {
        return '<li>' + e + '</li>';
      }).join('') + '</ul>';
      errorDiv.style.display = 'block';
      output.style.display = 'none';
      openPrBtn.style.display = 'none';
      copyBtn.style.display = 'none';
      return;
    }

    errorDiv.style.display = 'none';
    var yamlStr = generateYaml(values);
    output.textContent = yamlStr;
    output.style.display = 'block';
    openPrBtn.style.display = 'inline-block';
    copyBtn.style.display = 'inline-block';
  });

  copyBtn.addEventListener('click', function(e) {
    e.preventDefault();
    var text = output.textContent;
    navigator.clipboard.writeText(text).then(function() {
      var origText = copyBtn.textContent;
      copyBtn.textContent = 'Copied!';
      setTimeout(function() { copyBtn.textContent = origText; }, 2000);
    });
  });

  openPrBtn.addEventListener('click', function(e) {
    e.preventDefault();
    var yamlSnippet = output.textContent;
    // GitHub web editor URL — opens adopters.yaml for editing on rolling branch.
    var editUrl = 'https://github.com/ros2/ros2_documentation/edit/rolling/'
      + 'source/The-ROS2-Project/Adopters/adopters.yaml';
    // We cannot pre-fill the edit content via URL, but we copy the YAML to clipboard
    // and open the editor so the user can paste.
    navigator.clipboard.writeText(yamlSnippet).then(function() {
      window.open(editUrl, '_blank');
    }).catch(function() {
      // Fallback if clipboard fails.
      window.open(editUrl, '_blank');
    });
  });
}

/* ===== Init on page load ===== */

document.addEventListener('DOMContentLoaded', function() {
  initAdoptersTableFilters();
  initAdoptersForm();
});
