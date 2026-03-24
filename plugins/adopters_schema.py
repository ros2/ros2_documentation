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

"""
Shared schema constants and validation for the ROS 2 adopters YAML file.
This module has NO Sphinx dependency so it can be used in tests and CI scripts.

NOTE: The domain and status constants defined here must be kept in sync with
the corresponding values in source/_static/adopters.js (VALID_DOMAINS and
VALID_STATUSES). Any additions or changes must be applied to both files.
"""

VALID_DOMAINS = [
    'Agriculture',         # Farming, harvesting, crop monitoring, and precision agriculture
    'Aerial/Drone',        # UAVs, drones, aerial inspection, and survey systems
    'Automotive',          # Self-driving cars, ADAS, and ground vehicle autonomy
    'Construction',        # Site inspection, surveying, and construction automation
    'Consumer Robot',      # Home robots, entertainment robots, and personal companions
    'Defense/Government',  # Military, public safety, and national research programs
    'Education',           # University courses, student projects, and teaching platforms
    'Healthcare/Medical',  # Surgical robots, rehabilitation systems, and medical diagnostics
    'Humanoid',            # Bipedal and human-form robots
    'Logistics/Warehouse', # AMRs, inventory management, and last-mile delivery systems
    'Manufacturing',       # Industrial automation, assembly lines, and quality control
    'Marine',              # Underwater, surface, and coastal robotic systems
    'Research',            # General academic, laboratory, or experimental research
    'Space',               # Planetary rovers, orbital systems, and space exploration
    'Service Robot',       # Hospitality, cleaning, retail, and public-facing service robots
]

VALID_STATUSES = [
    'Active',     # Currently in active development or deployment
    'Maintained', # Deployed and stable, receiving only maintenance or bug fixes
    'Archived',   # No longer actively developed or deployed
    'PoC',        # Proof of concept, not yet in production
    'Research',   # Academic or experimental project, not intended for production
]

REQUIRED_FIELDS = [
    'organization',
    'project',
    'domain',
    'status',
    'country',
    'description'
]


def validate_adopters(adopters):
    """Validate a list of adopter entries. Returns list of error strings."""
    errors = []
    if not isinstance(adopters, list):
        return ["'adopters' must be a list"]
    for i, entry in enumerate(adopters):
        prefix = (
            f'Entry {i + 1} '
            f'({entry.get("organization", "unknown")}/{entry.get("project", "unknown")})'
        )
        if not isinstance(entry, dict):
            errors.append(f'{prefix}: must be a mapping')
            continue
        for field in REQUIRED_FIELDS:
            if field not in entry or not entry[field]:
                errors.append(f'{prefix}: missing required field "{field}"')
        if 'domain' in entry and entry['domain']:
            if not isinstance(entry['domain'], list):
                errors.append(f'{prefix}: "domain" must be a list')
            else:
                for d in entry['domain']:
                    if d not in VALID_DOMAINS:
                        errors.append(
                            f'{prefix}: invalid domain "{d}". '
                            f'Must be one of: {", ".join(VALID_DOMAINS)}'
                        )
        if 'status' in entry and entry['status']:
            if entry['status'] not in VALID_STATUSES:
                errors.append(
                    f'{prefix}: invalid status "{entry["status"]}". '
                    f'Must be one of: {", ".join(VALID_STATUSES)}'
                )
        if 'country' in entry and entry['country']:
            code = entry['country']
            if not isinstance(code, str) or len(code) != 2 or not code.isalpha():
                errors.append(
                    f'{prefix}: "country" must be a 2-letter ISO 3166-1 alpha-2 code, '
                    f'got "{code}"'
                )
    return errors
