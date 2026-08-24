/**
 * Populate ``.js-related-packages`` widgets from the rosdistro cache YAML.
 *
 * Depends on global ``pako`` (gzip) and ``yaml`` / ``jsyaml`` (js-yaml), loaded
 * earlier via html_js_files in conf.py.
 *
 * Data is loaded from up to three sources, tried in order:
 *   1. Proxy: an /api/rosdistro-cache/{distro}-cache.yaml.gz endpoint served
 *      from the same origin as the docs, for the freshest data. The path is
 *      baked in from conf.py. It is currently an assumption, because it exists
 *      only in local testing (tools/serve_docs_with_proxy.py). In production the
 *      hosting layer must provide this path. Alternatively, add a CORS header on
 *      repo.ros2.org so source 3 works and the proxy can be removed.
 *   2. Bundled fallback: the gzip snapshot downloaded into _static at build
 *      time. It is always on the same origin, so if the proxy is absent the list
 *      still renders, but only as fresh as the last docs build.
 *   3. Direct repo.ros2.org URL: a last resort that browsers usually block today
 *      because of CORS.
 */
(function () {
  'use strict';

  /** @type {Record<string, Promise<Record<string, string>>>} */
  var cacheByDistro = {};

  /**
   * Resolve the js-yaml API regardless of how the bundle exposes it.
   *
   * @returns {{ load: function(string): unknown }}
   */
  function yamlApi() {
    var g = typeof window !== 'undefined' ? window : globalThis;
    /* js-yaml UMD sets ``globalThis.jsyaml`` (see dist/js-yaml.min.js). */
    if (g.jsyaml && typeof g.jsyaml.load === 'function') {
      return g.jsyaml;
    }
    if (g.yaml && typeof g.yaml.load === 'function') {
      return g.yaml;
    }
    throw new Error('js-yaml is not loaded');
  }

  /**
   * Directory containing ``related_packages.js`` (ends with slash or empty).
   *
   * @returns {string}
   */
  function scriptBaseUrl() {
    var nodes = document.getElementsByTagName('script');
    var i;
    var src;
    for (i = nodes.length - 1; i >= 0; i--) {
      src = nodes[i].src;
      if (src && src.indexOf('related_packages.js') !== -1) {
        return src.replace(/related_packages\.js([?#].*)?$/i, '');
      }
    }
    return '';
  }

  /**
   * @param {string} distro
   * @returns {string|null}
   */
  function bundledCacheUrl(distro) {
    var base = scriptBaseUrl();
    if (!base) {
      return null;
    }
    return base + 'rosdistro_cache/' + distro + '-cache.yaml.gz';
  }

  /**
   * Prefer Sphinx-emitted ``data-bundled-cache-href`` (relative to page); then derive from script URL.
   *
   * @param {HTMLElement|null} widget
   * @param {string} distro
   * @returns {string|null}
   */
  function resolveBundledAbsoluteUrl(widget, distro) {
    var rel = widget && widget.getAttribute('data-bundled-cache-href');
    if (rel && typeof URL !== 'undefined') {
      try {
        return new URL(rel, window.location.href).href;
      } catch (e1) {
        /* ignore */
      }
    }
    return bundledCacheUrl(distro);
  }

  /**
   * Proxy URL configured by Sphinx via data attribute.
   *
   * @param {HTMLElement|null} widget
   * @param {string} distro
   * @returns {string|null}
   */
  function resolveProxyUrl(widget, distro) {
    var templateUrl = widget && widget.getAttribute('data-proxy-cache-href');
    if (!templateUrl) {
      return null;
    }
    return templateUrl.replace('{distro}', encodeURIComponent(distro));
  }

  /**
   * @param {string} distro
   * @param {HTMLElement|null} sampleWidget widget from this page (for data-bundled-cache-href)
   * @returns {Promise<Record<string, string>>}
   */
  function loadXmls(distro, sampleWidget) {
    var cacheKey =
      distro +
      '|' +
      (sampleWidget ? sampleWidget.getAttribute('data-proxy-cache-href') || '' : '') +
      '|' +
      (sampleWidget ? sampleWidget.getAttribute('data-bundled-cache-href') || '' : '');
    if (cacheByDistro[cacheKey]) {
      return cacheByDistro[cacheKey];
    }
    cacheByDistro[cacheKey] = fetchAndParse(
      distro,
      resolveProxyUrl(sampleWidget, distro),
      resolveBundledAbsoluteUrl(sampleWidget, distro)
    );
    return cacheByDistro[cacheKey];
  }

  /**
   * @param {string} distro
   * @param {string|null} proxyUrl backend proxy endpoint on the same origin (freshest)
   * @param {string|null} bundledAbsolute resolved URL on the same origin to the gzip, if any
   * @returns {Promise<Record<string, string>>}
   */
  function fetchAndParse(distro, proxyUrl, bundledAbsolute) {
    var remote =
      'https://repo.ros2.org/rosdistro_cache/' + encodeURIComponent(distro) + '-cache.yaml.gz';
    var urls = [];
    if (proxyUrl) {
      urls.push(proxyUrl);
    }
    if (bundledAbsolute) {
      urls.push(bundledAbsolute);
    }
    /* Final fallback may still fail in browsers due to upstream CORS. */
    urls.push(remote);

    return tryUrls(urls);
  }

  /**
   * @param {string[]} urls
   * @returns {Promise<Record<string, string>>}
   */
  function tryUrls(urls) {
    var i = 0;

    function next(lastErr) {
      if (i >= urls.length) {
        return Promise.reject(lastErr || new Error('failed to load rosdistro cache'));
      }
      var url = urls[i];
      i += 1;
      var controller = typeof AbortController !== 'undefined' ? new AbortController() : null;
      var timer = null;
      if (controller && i === 1) {
        /* Keep proxy attempt snappy so fallback isn't delayed. */
        timer = setTimeout(function () {
          controller.abort();
        }, 6000);
      }
      return fetch(url, { cache: 'no-cache', signal: controller ? controller.signal : undefined })
        .then(function (res) {
          if (timer) {
            clearTimeout(timer);
          }
          if (!res.ok) {
            throw new Error('HTTP ' + res.status + ' for ' + url);
          }
          return res.arrayBuffer();
        })
        .then(function (buf) {
          var g = typeof window !== 'undefined' ? window : globalThis;
          var inflated = g.pako.inflate(new Uint8Array(buf), { to: 'string' });
          var data = yamlApi().load(inflated);
          var xmls = data && data.release_package_xmls;
          if (!xmls || typeof xmls !== 'object') {
            throw new Error('release_package_xmls missing in rosdistro cache');
          }
          if (typeof console !== 'undefined' && console.info) {
            console.info('related_packages: loaded rosdistro cache from', url);
          }
          return /** @type {Record<string, string>} */ (xmls);
        })
        .catch(function (err) {
          if (timer) {
            clearTimeout(timer);
          }
          if (typeof console !== 'undefined' && console.warn) {
            console.warn('related_packages: failed', url, err);
          }
          /* Try next URL. For example a bundled 404, then the HTTPS remote, which may hit CORS. */
          return next(err);
        });
    }

    return next(null);
  }

  /**
   * Ordered, lowercased ``area`` tokens from a package's package.xml.
   *
   * Accepts either one ``<area>`` with values separated by commas or several
   * ``<area>`` elements. Duplicates are dropped and order is preserved.
   *
   * @param {string} xmlStr
   * @returns {string[]}
   */
  function extractAreaTokens(xmlStr) {
    var out = [];
    var re = /<area[^>]*>([^<]+)<\/area>/gi;
    var m;
    var parts;
    var k;
    var token;
    while ((m = re.exec(xmlStr)) !== null) {
      parts = m[1].split(',');
      for (k = 0; k < parts.length; k += 1) {
        token = parts[k].trim().toLowerCase();
        if (token && out.indexOf(token) === -1) {
          out.push(token);
        }
      }
    }
    return out;
  }

  /**
   * First (most specific) area value from a string of values separated by
   * commas, lowercased.
   *
   * @param {string} raw
   * @returns {string}
   */
  function primaryAreaFromString(raw) {
    var parts = (raw || '').split(',');
    var k;
    var token;
    for (k = 0; k < parts.length; k += 1) {
      token = parts[k].trim().toLowerCase();
      if (token) {
        return token;
      }
    }
    return '';
  }

  /**
   * A package matches when the page's primary area appears anywhere in the
   * package's ``<area>`` values.
   *
   * @param {string} xmlStr
   * @param {string} wantPrimary the page's primary (first) area value
   * @returns {boolean}
   */
  function matchesArea(xmlStr, wantPrimary) {
    if (!wantPrimary) {
      return false;
    }
    var tokens = extractAreaTokens(xmlStr);
    return tokens.indexOf(wantPrimary) !== -1;
  }

  /**
   * Core vs community scope from ``<related_scope>`` in package.xml export.
   * Defaults to community when the tag is absent (unknown upstream packages).
   *
   * @param {string} xmlStr
   * @returns {'core'|'community'}
   */
  function extractRelatedScope(xmlStr) {
    var match = /<related_scope\b[^>]*>([^<]+)<\/related_scope>/i.exec(xmlStr || '');
    if (!match) {
      return 'community';
    }
    var value = match[1].trim().toLowerCase();
    if (value === 'core' || value === 'community') {
      return value;
    }
    return 'community';
  }

  /**
   * @param {string} xmlStr
   * @returns {string}
   */
  function extractDescription(xmlStr) {
    if (typeof DOMParser !== 'undefined') {
      try {
        var doc = new DOMParser().parseFromString(xmlStr, 'application/xml');
        var parseErr = doc.getElementsByTagName('parsererror');
        if (!parseErr.length) {
          var nodes = doc.getElementsByTagName('description');
          if (nodes.length && nodes[0].textContent) {
            return nodes[0].textContent.replace(/\s+/g, ' ').trim();
          }
        }
      } catch (err) {
        /* Fall through to regex extraction. */
      }
    }

    var match = /<description\b[^>]*>([\s\S]*?)<\/description>/i.exec(xmlStr);
    if (!match) {
      return '';
    }
    return match[1]
      .replace(/<!\[CDATA\[([\s\S]*?)\]\]>/g, '$1')
      .replace(/<[^>]*>/g, ' ')
      .replace(/\s+/g, ' ')
      .trim();
  }

  /**
   * @param {string} distro
   * @param {string} pkg
   * @returns {string}
   */
  function docsPackageUrl(distro, pkg) {
    return (
      'https://docs.ros.org/en/' +
      encodeURIComponent(distro) +
      '/p/' +
      encodeURIComponent(pkg) +
      '/'
    );
  }

  /**
   * @param {HTMLElement} el
   * @param {Error} err
   */
  function showError(el, err) {
    el.classList.remove('related-packages--loading');
    el.classList.add('related-packages--error');
    el.innerHTML =
      '<p class="related-packages__status">Could not load package metadata. ' +
      'Rebuild the HTML documentation while online so the rosdistro cache is ' +
      'downloaded into <code>_static/rosdistro_cache/</code>, ' +
      'or check your network connection.</p>';
    if (typeof console !== 'undefined' && console.warn) {
      console.warn('related_packages:', err);
    }
  }

  /**
   * @param {HTMLAnchorElement} anchor
   * @returns {string|null}
   */
  function packageNameFromManualLink(anchor) {
    var href = anchor.getAttribute('href') || '';
    var match = /\/p\/([^/?#]+)\/?/.exec(href);
    if (match) {
      try {
        return decodeURIComponent(match[1]);
      } catch (e1) {
        return match[1];
      }
    }
    var text = (anchor.textContent || '').trim();
    if (text && text.indexOf(' ') === -1) {
      return text;
    }
    return null;
  }

  /**
   * Collect adjacent manual package bullets (name + list item).
   *
   * @param {HTMLUListElement|null} prevList
   * @param {HTMLUListElement|null} nextList
   * @returns {{ name: string, li: HTMLLIElement }[]}
   */
  function collectManualEntries(prevList, nextList) {
    var entries = [];

    function scan(ul) {
      var items;
      var i;
      var li;
      var anchor;
      var pkg;
      if (!ul) {
        return;
      }
      items = ul.children;
      for (i = 0; i < items.length; i += 1) {
        li = items[i];
        if (!li || li.tagName !== 'LI') {
          continue;
        }
        anchor = li.querySelector('a[href]');
        if (!anchor) {
          continue;
        }
        pkg = packageNameFromManualLink(anchor);
        if (pkg) {
          entries.push({ name: pkg, li: li });
        }
      }
    }

    scan(prevList);
    scan(nextList);
    return entries;
  }

  /**
   * Drop empty sibling manual lists left after absorption.
   *
   * @param {HTMLUListElement|null} ul
   * @returns {HTMLUListElement|null}
   */
  function pruneEmptyManualList(ul) {
    if (!ul) {
      return null;
    }
    if (!ul.querySelector('li')) {
      ul.remove();
      return null;
    }
    return ul;
  }

  /**
   * @param {string} pkg
   * @param {Record<string, string>} xmls
   * @param {string} distro
   * @param {boolean} collapsed
   * @returns {HTMLLIElement}
   */
  function createPackageListItem(pkg, xmls, distro, collapsed) {
    var li = document.createElement('li');
    var a = document.createElement('a');
    var description = extractDescription(xmls[pkg] || '');
    a.href = docsPackageUrl(distro, pkg);
    a.textContent = pkg;
    a.rel = 'noopener noreferrer';
    li.appendChild(a);
    li.appendChild(document.createTextNode(': ' + description));
    if (collapsed) {
      li.className = 'related-packages__item--extra';
      li.hidden = true;
    }
    return li;
  }

  /**
   * @param {HTMLUListElement} listEl
   * @param {number} hiddenCount
   * @param {HTMLElement|null} insertBeforeEl
   */
  function attachExpandControl(listEl, hiddenCount, insertBeforeEl) {
    var btn;
    var noun;
    var extras;
    var i;
    if (hiddenCount < 1) {
      return;
    }
    noun = hiddenCount === 1 ? 'package' : 'packages';
    btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'related-packages__expand';
    btn.setAttribute('aria-expanded', 'false');
    btn.setAttribute('aria-controls', listEl.id || '');
    if (!listEl.id) {
      listEl.id = 'related-packages-list-' + Math.random().toString(36).slice(2, 9);
      btn.setAttribute('aria-controls', listEl.id);
    }
    btn.textContent = 'Show ' + hiddenCount + ' more ' + noun;

    btn.addEventListener('click', function () {
      extras = listEl.querySelectorAll('.related-packages__item--extra');
      if (btn.getAttribute('aria-expanded') === 'true') {
        for (i = 0; i < extras.length; i += 1) {
          extras[i].hidden = true;
        }
        btn.setAttribute('aria-expanded', 'false');
        btn.textContent = 'Show ' + hiddenCount + ' more ' + noun;
      } else {
        for (i = 0; i < extras.length; i += 1) {
          extras[i].hidden = false;
        }
        btn.setAttribute('aria-expanded', 'true');
        btn.textContent = 'Show fewer';
      }
    });

    if (insertBeforeEl && insertBeforeEl.parentNode) {
      insertBeforeEl.parentNode.insertBefore(btn, insertBeforeEl);
    } else if (listEl.parentNode) {
      listEl.parentNode.insertBefore(btn, listEl.nextSibling);
    }
  }

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
   * Promote the author-written ``Related packages:`` paragraph to ``<h3>``.
   * Skips an optional adjacent manual ``<ul>`` between the intro and the widget.
   *
   * @param {HTMLElement} el the ``.js-related-packages`` widget
   */
  function promoteRelatedPackagesIntro(el) {
    var node = el.previousElementSibling;
    var h3;
    while (node && node.tagName === 'UL') {
      node = node.previousElementSibling;
    }
    if (!node || node.tagName !== 'P') {
      return;
    }
    if (stripTrailingColon(node.textContent).toLowerCase() !== 'related packages') {
      return;
    }
    h3 = document.createElement('h3');
    h3.className = 'related-packages__heading';
    h3.textContent = 'Related packages';
    node.parentNode.replaceChild(h3, node);
  }

  /**
   * @param {string} title
   * @returns {HTMLHeadingElement}
   */
  function createScopeHeading(title) {
    var h = document.createElement('h4');
    h.className = 'related-packages__scope-heading';
    h.textContent = stripTrailingColon(title);
    return h;
  }

  /**
   * Build one Core or Community list (alphabetical) with optional expand.
   *
   * @param {ParentNode} parent
   * @param {string} heading
   * @param {string[]} names
   * @param {Record<string, string>} xmls
   * @param {string} distro
   * @param {number} max
   * @param {number} visibleMax
   * @param {HTMLElement|null} insertBeforeEl
   */
  function appendScopedPackageList(
    parent,
    heading,
    names,
    xmls,
    distro,
    max,
    visibleMax,
    insertBeforeEl
  ) {
    var picked;
    var listEl;
    var j;
    var hiddenCount;
    if (!names.length) {
      return;
    }
    names = names.slice().sort(function (a, b) {
      return a.localeCompare(b);
    });
    picked = names.slice(0, max > 0 ? max : names.length);
    hiddenCount = picked.length > visibleMax ? picked.length - visibleMax : 0;

    parent.insertBefore(createScopeHeading(heading), insertBeforeEl);
    listEl = document.createElement('ul');
    listEl.className = 'related-packages__list';
    for (j = 0; j < picked.length; j += 1) {
      listEl.appendChild(
        createPackageListItem(picked[j], xmls, distro, j >= visibleMax)
      );
    }
    parent.insertBefore(listEl, insertBeforeEl);
    if (hiddenCount > 0) {
      attachExpandControl(listEl, hiddenCount, insertBeforeEl);
    }
  }

  /**
   * @param {HTMLElement} el
   * @param {Record<string, string>} xmls
   */
  function fillWidget(el, xmls) {
    var wantPrimary = primaryAreaFromString(el.getAttribute('data-area') || '');
    var max = parseInt(el.getAttribute('data-max') || '0', 10);
    var visibleMax = parseInt(el.getAttribute('data-visible-max') || '7', 10);
    var distro = el.getAttribute('data-distro') || 'rolling';
    var prevList = el.previousElementSibling;
    prevList = prevList && prevList.tagName === 'UL' ? prevList : null;
    var nextList = el.nextElementSibling;
    nextList = nextList && nextList.tagName === 'UL' ? nextList : null;
    var manuals = collectManualEntries(prevList, nextList);
    var matchedNames = Object.create(null);
    var parent = el.parentNode;
    var coreNames = [];
    var communityNames = [];
    var name;
    var xmlStr;
    var names;
    var i;
    var entry;

    // Area-matching packages (including any also listed manually).
    names = Object.keys(xmls).filter(function (pkgName) {
      xmlStr = xmls[pkgName];
      if (typeof xmlStr !== 'string') {
        return false;
      }
      return matchesArea(xmlStr, wantPrimary);
    });

    for (i = 0; i < names.length; i += 1) {
      name = names[i];
      matchedNames[name] = true;
      if (extractRelatedScope(xmls[name]) === 'core') {
        coreNames.push(name);
      } else {
        communityNames.push(name);
      }
    }

    // Manuals that match land in Core/Community (with description); remove
    // their plain bullets so they are not duplicated under "Related packages:".
    // Manuals that do not match stay under the author-written list.
    for (i = 0; i < manuals.length; i += 1) {
      entry = manuals[i];
      if (matchedNames[entry.name] && entry.li.parentNode) {
        entry.li.parentNode.removeChild(entry.li);
      }
    }

    el.classList.remove('related-packages--loading');

    prevList = pruneEmptyManualList(prevList);
    nextList = pruneEmptyManualList(nextList);

    if (prevList) {
      prevList.classList.add('related-packages__list');
    }
    if (nextList) {
      if (prevList) {
        while (nextList.firstChild) {
          prevList.appendChild(nextList.firstChild);
        }
        nextList.remove();
      } else {
        nextList.classList.add('related-packages__list');
        prevList = nextList;
        nextList = null;
      }
    }

    // Promote before inserting Core/Community so the intro is still adjacent.
    promoteRelatedPackagesIntro(el);

    if (!coreNames.length && !communityNames.length) {
      if (prevList) {
        el.remove();
        return;
      }
      el.innerHTML = '';
      var empty = document.createElement('p');
      empty.className = 'related-packages__empty';
      empty.textContent = 'No packages matched this filter.';
      el.appendChild(empty);
      return;
    }

    if (!parent) {
      return;
    }

    appendScopedPackageList(
      parent,
      'Core ROS packages',
      coreNames,
      xmls,
      distro,
      max,
      visibleMax,
      el
    );
    appendScopedPackageList(
      parent,
      'Community-contributed packages',
      communityNames,
      xmls,
      distro,
      max,
      visibleMax,
      el
    );
    el.remove();
  }

  function fillAll() {
    var widgets = document.querySelectorAll('.js-related-packages');
    if (!widgets.length) {
      return;
    }

    /** @type {Record<string, HTMLElement[]>} */
    var byDistro = {};
    var idx;
    for (idx = 0; idx < widgets.length; idx += 1) {
      var el = widgets[idx];
      var d = el.getAttribute('data-distro') || 'rolling';
      if (!byDistro[d]) {
        byDistro[d] = [];
      }
      byDistro[d].push(el);
    }

    var distroKeys = Object.keys(byDistro);
    var di;
    for (di = 0; di < distroKeys.length; di += 1) {
      (function (distro) {
        var group = byDistro[distro];
        loadXmls(distro, group[0]).then(
          function (xmls) {
            var gi;
            for (gi = 0; gi < group.length; gi += 1) {
              fillWidget(group[gi], xmls);
            }
          },
          function (err) {
            var ei;
            for (ei = 0; ei < group.length; ei += 1) {
              showError(group[ei], err);
            }
          }
        );
      })(distroKeys[di]);
    }
  }

  if (typeof document !== 'undefined') {
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', fillAll);
    } else {
      fillAll();
    }
  }
})();
