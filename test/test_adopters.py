# Copyright 2026 Sony Group Corporation.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for adopters YAML validation."""

import os
import sys

import yaml

# Make the plugins directory importable.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'plugins'))

from adopters_schema import VALID_DOMAINS, validate_adopters, validate_adopter_urls


def _valid_entry(**overrides):
    """Return a minimal valid adopter entry, with optional overrides."""
    entry = {
        'organization': 'Test Org',
        'project': 'Test Project',
        'domain': ['Research'],
        'date_added': '2026-03-25',
        'country': ['US'],
        'description': 'A test project.',
    }
    entry.update(overrides)
    return entry


class TestValidateAdopters:

    def test_valid_entry(self):
        assert validate_adopters([_valid_entry()]) == []

    def test_valid_entry_with_optional_fields(self):
        entry = _valid_entry(
            organization_url='https://example.com',
            project_url='https://example.com/project',
        )
        assert validate_adopters([entry]) == []

    def test_multiple_domains(self):
        entry = _valid_entry(domain=['Research', 'Education'])
        assert validate_adopters([entry]) == []

    def test_multiple_countries(self):
        entry = _valid_entry(country=['US', 'JP', 'DE'])
        assert validate_adopters([entry]) == []

    def test_missing_organization(self):
        entry = _valid_entry()
        del entry['organization']
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'organization' in errors[0]

    def test_missing_project(self):
        entry = _valid_entry()
        del entry['project']
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'project' in errors[0]

    def test_missing_domain(self):
        entry = _valid_entry()
        del entry['domain']
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'domain' in errors[0]

    def test_missing_date_added(self):
        entry = _valid_entry()
        del entry['date_added']
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'date_added' in errors[0]

    def test_missing_country(self):
        entry = _valid_entry()
        del entry['country']
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'country' in errors[0]

    def test_missing_description(self):
        entry = _valid_entry()
        del entry['description']
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'description' in errors[0]

    def test_invalid_domain(self):
        entry = _valid_entry(domain=['InvalidDomain'])
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'InvalidDomain' in errors[0]

    def test_invalid_date_added_year_only(self):
        entry = _valid_entry(date_added='2026')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'YYYY-MM-DD' in errors[0]

    def test_invalid_date_added_month_only(self):
        entry = _valid_entry(date_added='2026-03')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'YYYY-MM-DD' in errors[0]

    def test_invalid_date_added_bad_month(self):
        entry = _valid_entry(date_added='2026-13-01')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'YYYY-MM-DD' in errors[0]

    def test_invalid_date_added_bad_month_zero(self):
        entry = _valid_entry(date_added='2026-00-15')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'YYYY-MM-DD' in errors[0]

    def test_invalid_date_added_bad_day_zero(self):
        entry = _valid_entry(date_added='2026-03-00')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'YYYY-MM-DD' in errors[0]

    def test_invalid_date_added_bad_day_32(self):
        entry = _valid_entry(date_added='2026-03-32')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'YYYY-MM-DD' in errors[0]

    def test_valid_date_added_formats(self):
        for date in ['2020-01-01', '2026-12-31', '1999-06-15']:
            entry = _valid_entry(date_added=date)
            assert validate_adopters([entry]) == [], f'Date {date} should be valid'

    def test_country_not_a_list(self):
        entry = _valid_entry(country='US')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'list' in errors[0]

    def test_invalid_country_too_long(self):
        entry = _valid_entry(country=['USA'])
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'country' in errors[0]

    def test_invalid_country_numeric(self):
        entry = _valid_entry(country=['12'])
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'country' in errors[0]

    def test_invalid_country_in_list(self):
        entry = _valid_entry(country=['US', 'INVALID', 'JP'])
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'INVALID' in errors[0]

    def test_domain_not_a_list(self):
        entry = _valid_entry(domain='Research')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'list' in errors[0]

    def test_empty_list(self):
        assert validate_adopters([]) == []

    def test_not_a_list(self):
        errors = validate_adopters('not a list')
        assert len(errors) == 1
        assert 'must be a list' in errors[0]

    def test_all_valid_domains_accepted(self):
        for domain in VALID_DOMAINS:
            entry = _valid_entry(domain=[domain])
            assert validate_adopters([entry]) == [], f'Domain {domain} should be valid'

class TestValidateAdopterUrls:

    def test_no_urls_returns_empty(self):
        entry = _valid_entry()
        assert validate_adopter_urls([entry]) == []

    def test_valid_url(self):
        entry = _valid_entry(organization_url='https://www.google.com')
        errors = validate_adopter_urls([entry])
        assert errors == []

    def test_unreachable_url(self):
        entry = _valid_entry(
            organization_url='https://this-domain-does-not-exist-abc123xyz.example'
        )
        errors = validate_adopter_urls([entry])
        assert len(errors) == 1
        assert 'organization_url' in errors[0]
        assert 'not reachable' in errors[0]

    def test_not_a_list(self):
        errors = validate_adopter_urls('not a list')
        assert len(errors) == 1
        assert 'must be a list' in errors[0]

    def test_skips_non_dict_entries(self):
        errors = validate_adopter_urls(['not a dict'])
        assert errors == []


class TestAdoptersYamlFile:
    """Validate the actual adopters.yaml file."""

    def test_adopters_yaml_is_valid(self):
        yaml_path = os.path.join(
            os.path.dirname(__file__), '..', 'source',
            'The-ROS2-Project', 'Adopters', 'adopters.yaml',
        )
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        adopters = data.get('adopters', [])
        errors = validate_adopters(adopters)
        assert errors == [], f'Validation errors:\n' + '\n'.join(errors)
