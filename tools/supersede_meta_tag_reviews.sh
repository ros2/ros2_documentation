#!/usr/bin/env bash
# Minimise prior stamped summary reviews from ensure_meta_tags.py. Suggestion-carrying
# reviews (unstamped suggest-changes bodies) are left visible so Conversation keeps
# live inline suggestions until they are actioned.
set -euo pipefail

: "${GH_TOKEN:?GH_TOKEN is required}"
: "${REPOSITORY:?REPOSITORY is required}"
: "${PR_NUMBER:?PR_NUMBER is required}"

MARKER="${META_TAG_REVIEW_MARKER_ID:-ros2-meta-tags-ensure}"

echo "Marking prior meta-tag reviews as outdated."

reviews_json="$(gh api "repos/${REPOSITORY}/pulls/${PR_NUMBER}/reviews" --paginate)"
mapfile -t review_ids < <(
	echo "${reviews_json}" | jq -r \
		".[] | select(.body != null and (.body | contains(\"${MARKER}\"))) | .node_id"
)

minimize_query='mutation($subjectId: ID!) {
  minimizeComment(input: { subjectId: $subjectId, classifier: OUTDATED }) {
    minimizedComment { isMinimized }
  }
}'

for node_id in "${review_ids[@]:-}"; do
	[ -z "${node_id}" ] && continue
	gh api graphql -f query="${minimize_query}" -f subjectId="${node_id}" || true
done

echo "Superseded ${#review_ids[@]} prior review comment(s)."
