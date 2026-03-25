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

/* ===== ISO 3166-1 alpha-2 country codes ===== */

var ISO_COUNTRIES = [
  {code:'AD',name:'Andorra'},{code:'AE',name:'United Arab Emirates'},
  {code:'AF',name:'Afghanistan'},{code:'AG',name:'Antigua and Barbuda'},
  {code:'AL',name:'Albania'},{code:'AM',name:'Armenia'},
  {code:'AO',name:'Angola'},{code:'AR',name:'Argentina'},
  {code:'AT',name:'Austria'},{code:'AU',name:'Australia'},
  {code:'AZ',name:'Azerbaijan'},{code:'BA',name:'Bosnia and Herzegovina'},
  {code:'BB',name:'Barbados'},{code:'BD',name:'Bangladesh'},
  {code:'BE',name:'Belgium'},{code:'BF',name:'Burkina Faso'},
  {code:'BG',name:'Bulgaria'},{code:'BH',name:'Bahrain'},
  {code:'BI',name:'Burundi'},{code:'BJ',name:'Benin'},
  {code:'BN',name:'Brunei'},{code:'BO',name:'Bolivia'},
  {code:'BR',name:'Brazil'},{code:'BS',name:'Bahamas'},
  {code:'BT',name:'Bhutan'},{code:'BW',name:'Botswana'},
  {code:'BY',name:'Belarus'},{code:'BZ',name:'Belize'},
  {code:'CA',name:'Canada'},{code:'CD',name:'Congo (Democratic Republic)'},
  {code:'CF',name:'Central African Republic'},{code:'CG',name:'Congo'},
  {code:'CH',name:'Switzerland'},{code:'CI',name:"Cote d'Ivoire"},
  {code:'CL',name:'Chile'},{code:'CM',name:'Cameroon'},
  {code:'CN',name:'China'},{code:'CO',name:'Colombia'},
  {code:'CR',name:'Costa Rica'},{code:'CU',name:'Cuba'},
  {code:'CV',name:'Cape Verde'},{code:'CY',name:'Cyprus'},
  {code:'CZ',name:'Czechia'},{code:'DE',name:'Germany'},
  {code:'DJ',name:'Djibouti'},{code:'DK',name:'Denmark'},
  {code:'DM',name:'Dominica'},{code:'DO',name:'Dominican Republic'},
  {code:'DZ',name:'Algeria'},{code:'EC',name:'Ecuador'},
  {code:'EE',name:'Estonia'},{code:'EG',name:'Egypt'},
  {code:'ER',name:'Eritrea'},{code:'ES',name:'Spain'},
  {code:'ET',name:'Ethiopia'},{code:'FI',name:'Finland'},
  {code:'FJ',name:'Fiji'},{code:'FR',name:'France'},
  {code:'GA',name:'Gabon'},{code:'GB',name:'United Kingdom'},
  {code:'GD',name:'Grenada'},{code:'GE',name:'Georgia'},
  {code:'GH',name:'Ghana'},{code:'GM',name:'Gambia'},
  {code:'GN',name:'Guinea'},{code:'GQ',name:'Equatorial Guinea'},
  {code:'GR',name:'Greece'},{code:'GT',name:'Guatemala'},
  {code:'GW',name:'Guinea-Bissau'},{code:'GY',name:'Guyana'},
  {code:'HK',name:'Hong Kong'},{code:'HN',name:'Honduras'},
  {code:'HR',name:'Croatia'},{code:'HT',name:'Haiti'},
  {code:'HU',name:'Hungary'},{code:'ID',name:'Indonesia'},
  {code:'IE',name:'Ireland'},{code:'IL',name:'Israel'},
  {code:'IN',name:'India'},{code:'IQ',name:'Iraq'},
  {code:'IR',name:'Iran'},{code:'IS',name:'Iceland'},
  {code:'IT',name:'Italy'},{code:'JM',name:'Jamaica'},
  {code:'JO',name:'Jordan'},{code:'JP',name:'Japan'},
  {code:'KE',name:'Kenya'},{code:'KG',name:'Kyrgyzstan'},
  {code:'KH',name:'Cambodia'},{code:'KI',name:'Kiribati'},
  {code:'KM',name:'Comoros'},{code:'KN',name:'Saint Kitts and Nevis'},
  {code:'KP',name:'North Korea'},{code:'KR',name:'South Korea'},
  {code:'KW',name:'Kuwait'},{code:'KZ',name:'Kazakhstan'},
  {code:'LA',name:'Laos'},{code:'LB',name:'Lebanon'},
  {code:'LC',name:'Saint Lucia'},{code:'LI',name:'Liechtenstein'},
  {code:'LK',name:'Sri Lanka'},{code:'LR',name:'Liberia'},
  {code:'LS',name:'Lesotho'},{code:'LT',name:'Lithuania'},
  {code:'LU',name:'Luxembourg'},{code:'LV',name:'Latvia'},
  {code:'LY',name:'Libya'},{code:'MA',name:'Morocco'},
  {code:'MC',name:'Monaco'},{code:'MD',name:'Moldova'},
  {code:'ME',name:'Montenegro'},{code:'MG',name:'Madagascar'},
  {code:'MK',name:'North Macedonia'},{code:'ML',name:'Mali'},
  {code:'MM',name:'Myanmar'},{code:'MN',name:'Mongolia'},
  {code:'MR',name:'Mauritania'},{code:'MT',name:'Malta'},
  {code:'MU',name:'Mauritius'},{code:'MV',name:'Maldives'},
  {code:'MW',name:'Malawi'},{code:'MX',name:'Mexico'},
  {code:'MY',name:'Malaysia'},{code:'MZ',name:'Mozambique'},
  {code:'NA',name:'Namibia'},{code:'NE',name:'Niger'},
  {code:'NG',name:'Nigeria'},{code:'NI',name:'Nicaragua'},
  {code:'NL',name:'Netherlands'},{code:'NO',name:'Norway'},
  {code:'NP',name:'Nepal'},{code:'NR',name:'Nauru'},
  {code:'NZ',name:'New Zealand'},{code:'OM',name:'Oman'},
  {code:'PA',name:'Panama'},{code:'PE',name:'Peru'},
  {code:'PG',name:'Papua New Guinea'},{code:'PH',name:'Philippines'},
  {code:'PK',name:'Pakistan'},{code:'PL',name:'Poland'},
  {code:'PR',name:'Puerto Rico'},{code:'PS',name:'Palestine'},
  {code:'PT',name:'Portugal'},{code:'PW',name:'Palau'},
  {code:'PY',name:'Paraguay'},{code:'QA',name:'Qatar'},
  {code:'RO',name:'Romania'},{code:'RS',name:'Serbia'},
  {code:'RU',name:'Russia'},{code:'RW',name:'Rwanda'},
  {code:'SA',name:'Saudi Arabia'},{code:'SB',name:'Solomon Islands'},
  {code:'SC',name:'Seychelles'},{code:'SD',name:'Sudan'},
  {code:'SE',name:'Sweden'},{code:'SG',name:'Singapore'},
  {code:'SI',name:'Slovenia'},{code:'SK',name:'Slovakia'},
  {code:'SL',name:'Sierra Leone'},{code:'SM',name:'San Marino'},
  {code:'SN',name:'Senegal'},{code:'SO',name:'Somalia'},
  {code:'SR',name:'Suriname'},{code:'SS',name:'South Sudan'},
  {code:'SV',name:'El Salvador'},{code:'SY',name:'Syria'},
  {code:'SZ',name:'Eswatini'},{code:'TD',name:'Chad'},
  {code:'TG',name:'Togo'},{code:'TH',name:'Thailand'},
  {code:'TJ',name:'Tajikistan'},{code:'TL',name:'Timor-Leste'},
  {code:'TM',name:'Turkmenistan'},{code:'TN',name:'Tunisia'},
  {code:'TO',name:'Tonga'},{code:'TR',name:'Turkey'},
  {code:'TT',name:'Trinidad and Tobago'},{code:'TV',name:'Tuvalu'},
  {code:'TW',name:'Taiwan'},{code:'TZ',name:'Tanzania'},
  {code:'UA',name:'Ukraine'},{code:'UG',name:'Uganda'},
  {code:'US',name:'United States'},{code:'UY',name:'Uruguay'},
  {code:'UZ',name:'Uzbekistan'},{code:'VC',name:'Saint Vincent and the Grenadines'},
  {code:'VE',name:'Venezuela'},{code:'VN',name:'Vietnam'},
  {code:'VU',name:'Vanuatu'},{code:'WS',name:'Samoa'},
  {code:'YE',name:'Yemen'},{code:'ZA',name:'South Africa'},
  {code:'ZM',name:'Zambia'},{code:'ZW',name:'Zimbabwe'}
];

/* ===== Table filtering ===== */

function initAdoptersTableFilters() {
  var filterDomain = document.getElementById('adopters-filter-domain');
  var filterCountry = document.getElementById('adopters-filter-country');
  var filterSearch = document.getElementById('adopters-filter-search');
  var showAllToggle = document.getElementById('adopters-show-all');

  if (!filterDomain) return;  // Not on the adopters table page.

  // Compute the cutoff date (3 years ago) as YYYY-MM-DD string.
  var now = new Date();
  var cutoffYear = now.getFullYear() - 3;
  var cutoffMonth = String(now.getMonth() + 1).padStart(2, '0');
  var cutoffDay = String(now.getDate()).padStart(2, '0');
  var cutoffDate = cutoffYear + '-' + cutoffMonth + '-' + cutoffDay;

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

  // Auto-populate date_added with current YYYY-MM-DD.
  var dateField = document.getElementById('field-date-added');
  if (dateField) {
    var now = new Date();
    var yyyy = now.getFullYear();
    var mm = String(now.getMonth() + 1).padStart(2, '0');
    var dd = String(now.getDate()).padStart(2, '0');
    dateField.value = yyyy + '-' + mm + '-' + dd;
  }

  // Populate country dropdown from ISO_COUNTRIES list.
  var countrySelect = document.getElementById('field-country');
  var addCountryBtn = document.getElementById('adopters-add-country-btn');
  var selectedCountriesDiv = document.getElementById('adopters-selected-countries');
  var selectedCountries = [];

  if (countrySelect) {
    ISO_COUNTRIES.forEach(function(c) {
      var opt = document.createElement('option');
      opt.value = c.code;
      opt.textContent = c.name + ' (' + c.code + ')';
      countrySelect.appendChild(opt);
    });
  }

  function renderCountryTags() {
    selectedCountriesDiv.innerHTML = '';
    selectedCountries.forEach(function(code) {
      var tag = document.createElement('span');
      tag.className = 'adopters-country-tag';
      var entry = ISO_COUNTRIES.find(function(c) { return c.code === code; });
      var label = entry ? entry.name + ' (' + code + ')' : code;
      tag.textContent = label + ' ';
      var removeBtn = document.createElement('button');
      removeBtn.type = 'button';
      removeBtn.className = 'adopters-country-tag-remove';
      removeBtn.textContent = '\u00d7';
      removeBtn.addEventListener('click', function() {
        selectedCountries = selectedCountries.filter(function(c) { return c !== code; });
        renderCountryTags();
      });
      tag.appendChild(removeBtn);
      selectedCountriesDiv.appendChild(tag);
    });
  }

  if (addCountryBtn) {
    addCountryBtn.addEventListener('click', function(e) {
      e.preventDefault();
      var code = countrySelect.value;
      if (code && selectedCountries.indexOf(code) === -1) {
        selectedCountries.push(code);
        renderCountryTags();
      }
      countrySelect.value = '';
    });
  }

  function getFormValues() {
    var domains = [];
    var checkboxes = form.querySelectorAll('input[name="domain"]:checked');
    checkboxes.forEach(function(cb) { domains.push(cb.value); });

    return {
      organization: form.querySelector('#field-organization').value.trim(),
      organization_url: form.querySelector('#field-organization-url').value.trim(),
      project: form.querySelector('#field-project').value.trim(),
      project_url: form.querySelector('#field-project-url').value.trim(),
      domain: domains,
      date_added: form.querySelector('#field-date-added').value.trim(),
      country: selectedCountries.slice(),
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
    if (values.country.length === 0) errors.push('At least one country must be selected.');
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
