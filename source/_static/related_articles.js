/**
 * Promote the author-written ``Related articles:`` intro and expand long
 * build-time ``ul.related-articles`` lists.
 *
 * Items beyond the visible cap are marked ``related-articles__item--extra`` by
 * the Sphinx plugin. This script turns the intro paragraph into an ``h3`` and
 * inserts a ``Show N more articles`` control.
 */
(function () {
  'use strict';

  /**
   * Strip a trailing colon used in author-written intro labels.
   *
   * @param {string} text
   * @returns {string}
   */
  function stripTrailingColon(text) {
    return String(text || '')
      .replace(/\s+/g, ' ')
      .trim()
      .replace(/:+\s*$/, '');
  }

  /**
   * Promote the author-written ``Related articles:`` paragraph to ``<h3>``.
   * Skips an optional adjacent manual ``<ul>`` between the intro and the list.
   *
   * @param {HTMLUListElement} listEl
   */
  function promoteRelatedArticlesIntro(listEl) {
    var node = listEl.previousElementSibling;
    var h3;
    while (node && node.tagName === 'UL') {
      node = node.previousElementSibling;
    }
    if (!node || node.tagName !== 'P') {
      return;
    }
    if (stripTrailingColon(node.textContent).toLowerCase() !== 'related articles') {
      return;
    }
    h3 = document.createElement('h3');
    h3.className = 'related-articles__heading';
    h3.textContent = 'Related articles';
    node.parentNode.replaceChild(h3, node);
  }

  /**
   * @param {HTMLUListElement} listEl
   */
  function attachExpandControl(listEl) {
    var extras = listEl.querySelectorAll('li.related-articles__item--extra');
    var hiddenCount = extras.length;
    var btn;
    var noun;
    var i;

    if (hiddenCount < 1) {
      return;
    }
    if (listEl.parentNode && listEl.parentNode.querySelector(
      'button.related-articles__expand[data-for="' + (listEl.id || '') + '"]'
    )) {
      return;
    }

    noun = hiddenCount === 1 ? 'article' : 'articles';
    if (!listEl.id) {
      listEl.id = 'related-articles-list-' + Math.random().toString(36).slice(2, 9);
    }

    btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'related-articles__expand';
    btn.setAttribute('aria-expanded', 'false');
    btn.setAttribute('aria-controls', listEl.id);
    btn.setAttribute('data-for', listEl.id);
    btn.textContent = 'Show ' + hiddenCount + ' more ' + noun;

    btn.addEventListener('click', function () {
      if (btn.getAttribute('aria-expanded') === 'true') {
        listEl.classList.remove('is-expanded');
        for (i = 0; i < extras.length; i += 1) {
          extras[i].hidden = true;
        }
        btn.setAttribute('aria-expanded', 'false');
        btn.textContent = 'Show ' + hiddenCount + ' more ' + noun;
      } else {
        listEl.classList.add('is-expanded');
        for (i = 0; i < extras.length; i += 1) {
          extras[i].hidden = false;
        }
        btn.setAttribute('aria-expanded', 'true');
        btn.textContent = 'Show fewer';
      }
    });

    /* Start collapsed: ensure extras are hidden even if CSS failed to load. */
    for (i = 0; i < extras.length; i += 1) {
      extras[i].hidden = true;
    }

    if (listEl.parentNode) {
      listEl.parentNode.insertBefore(btn, listEl.nextSibling);
    }
  }

  function init() {
    var lists = document.querySelectorAll('ul.related-articles');
    var i;
    for (i = 0; i < lists.length; i += 1) {
      promoteRelatedArticlesIntro(lists[i]);
      attachExpandControl(lists[i]);
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
