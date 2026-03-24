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

from adopters_schema import VALID_DOMAINS, VALID_STATUSES, validate_adopters


def _valid_entry(**overrides):
    """Return a minimal valid adopter entry, with optional overrides."""
    entry = {
        'organization': 'Test Org',
        'project': 'Test Project',
        'domain': ['Research'],
        'status': 'Active',
        'country': 'US',
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

    def test_missing_status(self):
        entry = _valid_entry()
        del entry['status']
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'status' in errors[0]

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

    def test_invalid_status(self):
        entry = _valid_entry(status='InvalidStatus')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'InvalidStatus' in errors[0]

    def test_invalid_country_too_long(self):
        entry = _valid_entry(country='USA')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'country' in errors[0]

    def test_invalid_country_numeric(self):
        entry = _valid_entry(country='12')
        errors = validate_adopters([entry])
        assert len(errors) == 1
        assert 'country' in errors[0]

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

    def test_all_valid_statuses_accepted(self):
        for status in VALID_STATUSES:
            entry = _valid_entry(status=status)
            assert validate_adopters([entry]) == [], f'Status {status} should be valid'


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
