A Markdown Example
==================

Alots of this is taken from:

[http://www.unexpected-vortices.com/sw/rippledoc/quick-markdown-example.html](http://www.unexpected-vortices.com/sw/rippledoc/quick-markdown-example.html)

Some things have been commented out and annotated if they do not work.

----

Paragraphs are separated by a blank line.

2nd paragraph. *Italic*, **bold**, and `monospace`. Itemized lists
look like:

  * this one
  * that one
  * the other one

Note that --- not considering the asterisk --- the actual text
content starts at 4-columns in.

> Block quotes are
> written like so.
>
> They can span multiple paragraphs,
> if you like.

Use 3 dashes for an em-dash. Use 2 dashes for ranges (ex., "it's all
in chapters 12--14"). Three dots ... will be converted to an ellipsis.
Unicode is supported. ☺

An h2 header
------------

Here's a numbered list:

 1. first item
 2. second item
 3. third item

Note again how the actual text starts at 4 columns in (4 characters
from the left side). Here's a code sample:

    # Let me re-iterate ...
    for i in 1 .. 10 { do-something(i) }

As you probably guessed, indented 4 spaces. By the way, instead of
indenting the block, you can use delimited blocks, if you like:

~~~
define foobar() {
    print "Welcome to flavor country!";
}
~~~

(which makes copying & pasting easier). You can optionally mark the
delimited block for Pandoc to syntax highlight it:

~~~python
import time
# Quick, count to ten!
for i in range(10):
    # (but not *too* quick)
    time.sleep(0.5)
    print(i)
~~~



### An h3 header ###

Now a nested list:

 1. First, get these ingredients:

      * carrots
      * celery
      * lentils

 2. Boil some water.

 3. Dump everything in the pot and follow
    this algorithm:

        find wooden spoon
        uncover pot
        stir
        cover pot
        balance wooden spoon precariously on pot handle
        wait 10 minutes
        goto first step (or shut off burner when done)

    Do not bump wooden spoon or it will fall.

Notice again how text always lines up on 4-space indents (including
that last line which continues item 3 above).

Here's a link to [a website](http://foo.bar).

And to a [local doc (about)](../About).

And to a [section heading in the current doc](#an-h2-header).

**Renders but does not work as expected:**

```
Here's a footnote [^1].

[^1]: Some footnote text.
```

**Renders but does not work as expected (see below for how to use rst tables instead):**

```
Tables can look like this:

Name           Size  Material      Color
------------- -----  ------------  ------------
All Business      9  leather       brown
Roundabout       10  hemp canvas   natural
Cinderella       11  glass         transparent

Table: Shoes sizes, materials, and colors.

(The above is the caption for the table.) Pandoc also supports
multi-line tables:

--------  -----------------------
Keyword   Text
--------  -----------------------
red       Sunsets, apples, and
          other red or reddish
          things.

green     Leaves, grass, frogs
          and other things it's
          not easy being.
--------  -----------------------

A horizontal rule follows.
```

***

**Renders but does not work as expected:**

```
Here's a definition list:

apples
  : Good for making applesauce.

oranges
  : Citrus!

tomatoes
  : There's no "e" in tomatoe.

Again, text is indented 4 spaces. (Put a blank line between each
term and  its definition to spread things out more.)
```

**Renders but does not work as expected:**

```
Here's a "line block" (note how whitespace is honored):

| Line one
|   Line too
| Line tree
```

and images can be specified like so:

*![Free for commercial use, no attribution required, from: https://pixabay.com/photos/kittens-cat-cat-puppy-rush-555822/](example-image.jpg "alt text")*

**Renders but does not work as expected:**

```
Inline math equation: $\omega = d\phi / dt$. Display
math should get its own line like so:

$$I = \int \rho R^{2} dV$$

```

And note that you can backslash-escape any punctuation characters
which you wish to be displayed literally, ex.: \`foo\`, \*bar\*, etc.

----

More Complicated Examples
=========================

This is mostly taken from [https://recommonmark.readthedocs.io/en/latest/auto_structify.html#configuring-autostructify](https://recommonmark.readthedocs.io/en/latest/auto_structify.html#configuring-autostructify), but there are even more examples there.

You can embed ReST:

```eval_rst
=====  =====  ======
   Inputs     Output
------------  ------
  A      B    A or B
=====  =====  ======
False  False  False
True   False  True
False  True   True
True   True   True
=====  =====  ======
```

Or like this:

``` important:: Its a note! in markdown!
```

### Code Blocks ###

Normal triple backticks work:

```python
def function():
    return True
```

### An Advanced Example ###

``` sidebar:: Line numbers and highlights

     emphasis-lines:
       highlights the lines.
     linenos:
       shows the line numbers as well.
     caption:
       shown at the top of the code block.
     name:
       may be referenced with ``:ref:`` later.
```

``` code-block::
    :linenos:
    :emphasize-lines: 3,5
    :caption: An example code-block with everything turned on.
    :name: Full code-block example

    # Comment line
    import System
    System.run_emphasis_line
    # Long lines in code blocks create a auto horizontal scrollbar
    System.exit!
```

The `<div style="clear: right;"></div>` line clears the sidebar for the next title.

<div style="clear: right;"></div>

### Math Example ###

Math does work if you use this style:

```math
$ y=\sum_{i=1}^n g(x_i) $
```
