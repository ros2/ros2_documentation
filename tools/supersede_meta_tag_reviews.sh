#!/usr/bin/env bash
# Minimise prior bot PR reviews stamped by ensure_meta_tags.py and set should_post.
set -euo pipefail

: "${GH_TOKEN:?GH_TOKEN is required}"
: "${REPOSITORY:?REPOSITORY is required}"
: "${PR_NUMBER:?PR_NUMBER is required}"
: "${HAS_RESULTS:?HAS_RESULTS is required}"

MARKER="${META_TAG_REVIEW_MARKER_ID:-ros2-meta-tags-ensure}"
STATUS_FILE="${STATUS_FILE:-${GITHUB_OUTPUT:-}}"

if [ -z "${STATUS_FILE}" ]; then
	echo "STATUS_FILE or GITHUB_OUTPUT is required" >&2
	exit 1
fi

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

if [ "${HAS_RESULTS}" = "true" ]; then
	echo "should_post=true" >>"${STATUS_FILE}"
else
	echo "All meta-tag issues resolved; not posting a new review."
	echo "should_post=false" >>"${STATUS_FILE}"
fi
