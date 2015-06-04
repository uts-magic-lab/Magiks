<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.1//EN" "http://www.w3.org/TR/xhtml11/DTD/xhtml11.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="en-US">
<head>
<link rel="icon" href="/cpython/static/hgicon.png" type="image/png" />
<meta name="robots" content="index, nofollow" />
<link rel="stylesheet" href="/cpython/static/style-paper.css" type="text/css" />
<script type="text/javascript" src="/cpython/static/mercurial.js"></script>

<link rel="stylesheet" href="/cpython/highlightcss" type="text/css" />
<title>cpython: 69951573cb0e Lib/statistics.py</title>
</head>
<body>

<div class="container">
<div class="menu">
<div class="logo">
<a href="https://hg.python.org">
<img src="/cpython/static/hglogo.png" alt="back to hg.python.org repositories" /></a>
</div>
<ul>
<li><a href="/cpython/shortlog/69951573cb0e">log</a></li>
<li><a href="/cpython/graph/69951573cb0e">graph</a></li>
<li><a href="/cpython/tags">tags</a></li>
<li><a href="/cpython/branches">branches</a></li>
</ul>
<ul>
<li><a href="/cpython/rev/69951573cb0e">changeset</a></li>
<li><a href="/cpython/file/69951573cb0e/Lib/">browse</a></li>
</ul>
<ul>
<li class="active">file</li>
<li><a href="/cpython/file/tip/Lib/statistics.py">latest</a></li>
<li><a href="/cpython/diff/69951573cb0e/Lib/statistics.py">diff</a></li>
<li><a href="/cpython/comparison/69951573cb0e/Lib/statistics.py">comparison</a></li>
<li><a href="/cpython/annotate/69951573cb0e/Lib/statistics.py">annotate</a></li>
<li><a href="/cpython/log/69951573cb0e/Lib/statistics.py">file log</a></li>
<li><a href="/cpython/raw-file/69951573cb0e/Lib/statistics.py">raw</a></li>
</ul>
<ul>
<li><a href="/cpython/help">help</a></li>
</ul>
</div>

<div class="main">
<h2 class="breadcrumb"><a href="/">Mercurial</a> &gt; <a href="/cpython">cpython</a> </h2>
<h3>view Lib/statistics.py @ 95819:69951573cb0e</h3>

<form class="search" action="/cpython/log">

<p><input name="rev" id="search1" type="text" size="30" /></p>
<div id="hint">Find changesets by keywords (author, files, the commit message), revision
number or hash, or <a href="/cpython/help/revsets">revset expression</a>.</div>
</form>

<div class="description">Issue #21354: PyCFunction_New function is exposed by python DLL again.</a> [<a href="http://bugs.python.org/21354" class="issuelink">#21354</a>]</div>

<table id="changesetEntry">
<tr>
 <th class="author">author</th>
 <td class="author">&#65;&#110;&#100;&#114;&#101;&#119;&#32;&#83;&#118;&#101;&#116;&#108;&#111;&#118;&#32;&#60;&#97;&#110;&#100;&#114;&#101;&#119;&#46;&#115;&#118;&#101;&#116;&#108;&#111;&#118;&#64;&#103;&#109;&#97;&#105;&#108;&#46;&#99;&#111;&#109;&#62;</td>
</tr>
<tr>
 <th class="date">date</th>
 <td class="date age">Mon, 27 Apr 2015 17:48:50 +0300</td>
</tr>
<tr>
 <th class="author">parents</th>
 <td class="author"><a href="/cpython/file/5db74cd953ab/Lib/statistics.py">5db74cd953ab</a> </td>
</tr>
<tr>
 <th class="author">children</th>
 <td class="author"><a href="/cpython/file/4480506137ed/Lib/statistics.py">4480506137ed</a> </td>
</tr>
</table>

<div class="overflow">
<div class="sourcefirst linewraptoggle">line wrap: <a class="linewraplink" href="javascript:toggleLinewrap()">on</a></div>
<div class="sourcefirst"> line source</div>
<pre class="sourcelines stripes4 wrap">
<span id="l1"><span class="c">##  Module statistics.py</span></span><a href="#l1"></a>
<span id="l2"><span class="c">##</span></span><a href="#l2"></a>
<span id="l3"><span class="c">##  Copyright (c) 2013 Steven D&#39;Aprano &lt;steve+python@pearwood.info&gt;.</span></span><a href="#l3"></a>
<span id="l4"><span class="c">##</span></span><a href="#l4"></a>
<span id="l5"><span class="c">##  Licensed under the Apache License, Version 2.0 (the &quot;License&quot;);</span></span><a href="#l5"></a>
<span id="l6"><span class="c">##  you may not use this file except in compliance with the License.</span></span><a href="#l6"></a>
<span id="l7"><span class="c">##  You may obtain a copy of the License at</span></span><a href="#l7"></a>
<span id="l8"><span class="c">##</span></span><a href="#l8"></a>
<span id="l9"><span class="c">##  http://www.apache.org/licenses/LICENSE-2.0</span></span><a href="#l9"></a>
<span id="l10"><span class="c">##</span></span><a href="#l10"></a>
<span id="l11"><span class="c">##  Unless required by applicable law or agreed to in writing, software</span></span><a href="#l11"></a>
<span id="l12"><span class="c">##  distributed under the License is distributed on an &quot;AS IS&quot; BASIS,</span></span><a href="#l12"></a>
<span id="l13"><span class="c">##  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.</span></span><a href="#l13"></a>
<span id="l14"><span class="c">##  See the License for the specific language governing permissions and</span></span><a href="#l14"></a>
<span id="l15"><span class="c">##  limitations under the License.</span></span><a href="#l15"></a>
<span id="l16"></span><a href="#l16"></a>
<span id="l17"></span><a href="#l17"></a>
<span id="l18"><span class="sd">&quot;&quot;&quot;</span></span><a href="#l18"></a>
<span id="l19"><span class="sd">Basic statistics module.</span></span><a href="#l19"></a>
<span id="l20"></span><a href="#l20"></a>
<span id="l21"><span class="sd">This module provides functions for calculating statistics of data, including</span></span><a href="#l21"></a>
<span id="l22"><span class="sd">averages, variance, and standard deviation.</span></span><a href="#l22"></a>
<span id="l23"></span><a href="#l23"></a>
<span id="l24"><span class="sd">Calculating averages</span></span><a href="#l24"></a>
<span id="l25"><span class="sd">--------------------</span></span><a href="#l25"></a>
<span id="l26"></span><a href="#l26"></a>
<span id="l27"><span class="sd">==================  =============================================</span></span><a href="#l27"></a>
<span id="l28"><span class="sd">Function            Description</span></span><a href="#l28"></a>
<span id="l29"><span class="sd">==================  =============================================</span></span><a href="#l29"></a>
<span id="l30"><span class="sd">mean                Arithmetic mean (average) of data.</span></span><a href="#l30"></a>
<span id="l31"><span class="sd">median              Median (middle value) of data.</span></span><a href="#l31"></a>
<span id="l32"><span class="sd">median_low          Low median of data.</span></span><a href="#l32"></a>
<span id="l33"><span class="sd">median_high         High median of data.</span></span><a href="#l33"></a>
<span id="l34"><span class="sd">median_grouped      Median, or 50th percentile, of grouped data.</span></span><a href="#l34"></a>
<span id="l35"><span class="sd">mode                Mode (most common value) of data.</span></span><a href="#l35"></a>
<span id="l36"><span class="sd">==================  =============================================</span></span><a href="#l36"></a>
<span id="l37"></span><a href="#l37"></a>
<span id="l38"><span class="sd">Calculate the arithmetic mean (&quot;the average&quot;) of data:</span></span><a href="#l38"></a>
<span id="l39"></span><a href="#l39"></a>
<span id="l40"><span class="sd">&gt;&gt;&gt; mean([-1.0, 2.5, 3.25, 5.75])</span></span><a href="#l40"></a>
<span id="l41"><span class="sd">2.625</span></span><a href="#l41"></a>
<span id="l42"></span><a href="#l42"></a>
<span id="l43"></span><a href="#l43"></a>
<span id="l44"><span class="sd">Calculate the standard median of discrete data:</span></span><a href="#l44"></a>
<span id="l45"></span><a href="#l45"></a>
<span id="l46"><span class="sd">&gt;&gt;&gt; median([2, 3, 4, 5])</span></span><a href="#l46"></a>
<span id="l47"><span class="sd">3.5</span></span><a href="#l47"></a>
<span id="l48"></span><a href="#l48"></a>
<span id="l49"></span><a href="#l49"></a>
<span id="l50"><span class="sd">Calculate the median, or 50th percentile, of data grouped into class intervals</span></span><a href="#l50"></a>
<span id="l51"><span class="sd">centred on the data values provided. E.g. if your data points are rounded to</span></span><a href="#l51"></a>
<span id="l52"><span class="sd">the nearest whole number:</span></span><a href="#l52"></a>
<span id="l53"></span><a href="#l53"></a>
<span id="l54"><span class="sd">&gt;&gt;&gt; median_grouped([2, 2, 3, 3, 3, 4])  #doctest: +ELLIPSIS</span></span><a href="#l54"></a>
<span id="l55"><span class="sd">2.8333333333...</span></span><a href="#l55"></a>
<span id="l56"></span><a href="#l56"></a>
<span id="l57"><span class="sd">This should be interpreted in this way: you have two data points in the class</span></span><a href="#l57"></a>
<span id="l58"><span class="sd">interval 1.5-2.5, three data points in the class interval 2.5-3.5, and one in</span></span><a href="#l58"></a>
<span id="l59"><span class="sd">the class interval 3.5-4.5. The median of these data points is 2.8333...</span></span><a href="#l59"></a>
<span id="l60"></span><a href="#l60"></a>
<span id="l61"></span><a href="#l61"></a>
<span id="l62"><span class="sd">Calculating variability or spread</span></span><a href="#l62"></a>
<span id="l63"><span class="sd">---------------------------------</span></span><a href="#l63"></a>
<span id="l64"></span><a href="#l64"></a>
<span id="l65"><span class="sd">==================  =============================================</span></span><a href="#l65"></a>
<span id="l66"><span class="sd">Function            Description</span></span><a href="#l66"></a>
<span id="l67"><span class="sd">==================  =============================================</span></span><a href="#l67"></a>
<span id="l68"><span class="sd">pvariance           Population variance of data.</span></span><a href="#l68"></a>
<span id="l69"><span class="sd">variance            Sample variance of data.</span></span><a href="#l69"></a>
<span id="l70"><span class="sd">pstdev              Population standard deviation of data.</span></span><a href="#l70"></a>
<span id="l71"><span class="sd">stdev               Sample standard deviation of data.</span></span><a href="#l71"></a>
<span id="l72"><span class="sd">==================  =============================================</span></span><a href="#l72"></a>
<span id="l73"></span><a href="#l73"></a>
<span id="l74"><span class="sd">Calculate the standard deviation of sample data:</span></span><a href="#l74"></a>
<span id="l75"></span><a href="#l75"></a>
<span id="l76"><span class="sd">&gt;&gt;&gt; stdev([2.5, 3.25, 5.5, 11.25, 11.75])  #doctest: +ELLIPSIS</span></span><a href="#l76"></a>
<span id="l77"><span class="sd">4.38961843444...</span></span><a href="#l77"></a>
<span id="l78"></span><a href="#l78"></a>
<span id="l79"><span class="sd">If you have previously calculated the mean, you can pass it as the optional</span></span><a href="#l79"></a>
<span id="l80"><span class="sd">second argument to the four &quot;spread&quot; functions to avoid recalculating it:</span></span><a href="#l80"></a>
<span id="l81"></span><a href="#l81"></a>
<span id="l82"><span class="sd">&gt;&gt;&gt; data = [1, 2, 2, 4, 4, 4, 5, 6]</span></span><a href="#l82"></a>
<span id="l83"><span class="sd">&gt;&gt;&gt; mu = mean(data)</span></span><a href="#l83"></a>
<span id="l84"><span class="sd">&gt;&gt;&gt; pvariance(data, mu)</span></span><a href="#l84"></a>
<span id="l85"><span class="sd">2.5</span></span><a href="#l85"></a>
<span id="l86"></span><a href="#l86"></a>
<span id="l87"></span><a href="#l87"></a>
<span id="l88"><span class="sd">Exceptions</span></span><a href="#l88"></a>
<span id="l89"><span class="sd">----------</span></span><a href="#l89"></a>
<span id="l90"></span><a href="#l90"></a>
<span id="l91"><span class="sd">A single exception is defined: StatisticsError is a subclass of ValueError.</span></span><a href="#l91"></a>
<span id="l92"></span><a href="#l92"></a>
<span id="l93"><span class="sd">&quot;&quot;&quot;</span></span><a href="#l93"></a>
<span id="l94"></span><a href="#l94"></a>
<span id="l95"><span class="n">__all__</span> <span class="o">=</span> <span class="p">[</span> <span class="s">&#39;StatisticsError&#39;</span><span class="p">,</span></span><a href="#l95"></a>
<span id="l96">            <span class="s">&#39;pstdev&#39;</span><span class="p">,</span> <span class="s">&#39;pvariance&#39;</span><span class="p">,</span> <span class="s">&#39;stdev&#39;</span><span class="p">,</span> <span class="s">&#39;variance&#39;</span><span class="p">,</span></span><a href="#l96"></a>
<span id="l97">            <span class="s">&#39;median&#39;</span><span class="p">,</span>  <span class="s">&#39;median_low&#39;</span><span class="p">,</span> <span class="s">&#39;median_high&#39;</span><span class="p">,</span> <span class="s">&#39;median_grouped&#39;</span><span class="p">,</span></span><a href="#l97"></a>
<span id="l98">            <span class="s">&#39;mean&#39;</span><span class="p">,</span> <span class="s">&#39;mode&#39;</span><span class="p">,</span></span><a href="#l98"></a>
<span id="l99">          <span class="p">]</span></span><a href="#l99"></a>
<span id="l100"></span><a href="#l100"></a>
<span id="l101"></span><a href="#l101"></a>
<span id="l102"><span class="kn">import</span> <span class="nn">collections</span></span><a href="#l102"></a>
<span id="l103"><span class="kn">import</span> <span class="nn">math</span></span><a href="#l103"></a>
<span id="l104"></span><a href="#l104"></a>
<span id="l105"><span class="kn">from</span> <span class="nn">fractions</span> <span class="kn">import</span> <span class="n">Fraction</span></span><a href="#l105"></a>
<span id="l106"><span class="kn">from</span> <span class="nn">decimal</span> <span class="kn">import</span> <span class="n">Decimal</span></span><a href="#l106"></a>
<span id="l107"></span><a href="#l107"></a>
<span id="l108"></span><a href="#l108"></a>
<span id="l109"><span class="c"># === Exceptions ===</span></span><a href="#l109"></a>
<span id="l110"></span><a href="#l110"></a>
<span id="l111"><span class="k">class</span> <span class="nc">StatisticsError</span><span class="p">(</span><span class="ne">ValueError</span><span class="p">):</span></span><a href="#l111"></a>
<span id="l112">    <span class="k">pass</span></span><a href="#l112"></a>
<span id="l113"></span><a href="#l113"></a>
<span id="l114"></span><a href="#l114"></a>
<span id="l115"><span class="c"># === Private utilities ===</span></span><a href="#l115"></a>
<span id="l116"></span><a href="#l116"></a>
<span id="l117"><span class="k">def</span> <span class="nf">_sum</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">start</span><span class="o">=</span><span class="mi">0</span><span class="p">):</span></span><a href="#l117"></a>
<span id="l118">    <span class="sd">&quot;&quot;&quot;_sum(data [, start]) -&gt; value</span></span><a href="#l118"></a>
<span id="l119"></span><a href="#l119"></a>
<span id="l120"><span class="sd">    Return a high-precision sum of the given numeric data. If optional</span></span><a href="#l120"></a>
<span id="l121"><span class="sd">    argument ``start`` is given, it is added to the total. If ``data`` is</span></span><a href="#l121"></a>
<span id="l122"><span class="sd">    empty, ``start`` (defaulting to 0) is returned.</span></span><a href="#l122"></a>
<span id="l123"></span><a href="#l123"></a>
<span id="l124"></span><a href="#l124"></a>
<span id="l125"><span class="sd">    Examples</span></span><a href="#l125"></a>
<span id="l126"><span class="sd">    --------</span></span><a href="#l126"></a>
<span id="l127"></span><a href="#l127"></a>
<span id="l128"><span class="sd">    &gt;&gt;&gt; _sum([3, 2.25, 4.5, -0.5, 1.0], 0.75)</span></span><a href="#l128"></a>
<span id="l129"><span class="sd">    11.0</span></span><a href="#l129"></a>
<span id="l130"></span><a href="#l130"></a>
<span id="l131"><span class="sd">    Some sources of round-off error will be avoided:</span></span><a href="#l131"></a>
<span id="l132"></span><a href="#l132"></a>
<span id="l133"><span class="sd">    &gt;&gt;&gt; _sum([1e50, 1, -1e50] * 1000)  # Built-in sum returns zero.</span></span><a href="#l133"></a>
<span id="l134"><span class="sd">    1000.0</span></span><a href="#l134"></a>
<span id="l135"></span><a href="#l135"></a>
<span id="l136"><span class="sd">    Fractions and Decimals are also supported:</span></span><a href="#l136"></a>
<span id="l137"></span><a href="#l137"></a>
<span id="l138"><span class="sd">    &gt;&gt;&gt; from fractions import Fraction as F</span></span><a href="#l138"></a>
<span id="l139"><span class="sd">    &gt;&gt;&gt; _sum([F(2, 3), F(7, 5), F(1, 4), F(5, 6)])</span></span><a href="#l139"></a>
<span id="l140"><span class="sd">    Fraction(63, 20)</span></span><a href="#l140"></a>
<span id="l141"></span><a href="#l141"></a>
<span id="l142"><span class="sd">    &gt;&gt;&gt; from decimal import Decimal as D</span></span><a href="#l142"></a>
<span id="l143"><span class="sd">    &gt;&gt;&gt; data = [D(&quot;0.1375&quot;), D(&quot;0.2108&quot;), D(&quot;0.3061&quot;), D(&quot;0.0419&quot;)]</span></span><a href="#l143"></a>
<span id="l144"><span class="sd">    &gt;&gt;&gt; _sum(data)</span></span><a href="#l144"></a>
<span id="l145"><span class="sd">    Decimal(&#39;0.6963&#39;)</span></span><a href="#l145"></a>
<span id="l146"></span><a href="#l146"></a>
<span id="l147"><span class="sd">    Mixed types are currently treated as an error, except that int is</span></span><a href="#l147"></a>
<span id="l148"><span class="sd">    allowed.</span></span><a href="#l148"></a>
<span id="l149"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l149"></a>
<span id="l150">    <span class="c"># We fail as soon as we reach a value that is not an int or the type of</span></span><a href="#l150"></a>
<span id="l151">    <span class="c"># the first value which is not an int. E.g. _sum([int, int, float, int])</span></span><a href="#l151"></a>
<span id="l152">    <span class="c"># is okay, but sum([int, int, float, Fraction]) is not.</span></span><a href="#l152"></a>
<span id="l153">    <span class="n">allowed_types</span> <span class="o">=</span> <span class="nb">set</span><span class="p">([</span><span class="nb">int</span><span class="p">,</span> <span class="nb">type</span><span class="p">(</span><span class="n">start</span><span class="p">)])</span></span><a href="#l153"></a>
<span id="l154">    <span class="n">n</span><span class="p">,</span> <span class="n">d</span> <span class="o">=</span> <span class="n">_exact_ratio</span><span class="p">(</span><span class="n">start</span><span class="p">)</span></span><a href="#l154"></a>
<span id="l155">    <span class="n">partials</span> <span class="o">=</span> <span class="p">{</span><span class="n">d</span><span class="p">:</span> <span class="n">n</span><span class="p">}</span>  <span class="c"># map {denominator: sum of numerators}</span></span><a href="#l155"></a>
<span id="l156">    <span class="c"># Micro-optimizations.</span></span><a href="#l156"></a>
<span id="l157">    <span class="n">exact_ratio</span> <span class="o">=</span> <span class="n">_exact_ratio</span></span><a href="#l157"></a>
<span id="l158">    <span class="n">partials_get</span> <span class="o">=</span> <span class="n">partials</span><span class="o">.</span><span class="n">get</span></span><a href="#l158"></a>
<span id="l159">    <span class="c"># Add numerators for each denominator.</span></span><a href="#l159"></a>
<span id="l160">    <span class="k">for</span> <span class="n">x</span> <span class="ow">in</span> <span class="n">data</span><span class="p">:</span></span><a href="#l160"></a>
<span id="l161">        <span class="n">_check_type</span><span class="p">(</span><span class="nb">type</span><span class="p">(</span><span class="n">x</span><span class="p">),</span> <span class="n">allowed_types</span><span class="p">)</span></span><a href="#l161"></a>
<span id="l162">        <span class="n">n</span><span class="p">,</span> <span class="n">d</span> <span class="o">=</span> <span class="n">exact_ratio</span><span class="p">(</span><span class="n">x</span><span class="p">)</span></span><a href="#l162"></a>
<span id="l163">        <span class="n">partials</span><span class="p">[</span><span class="n">d</span><span class="p">]</span> <span class="o">=</span> <span class="n">partials_get</span><span class="p">(</span><span class="n">d</span><span class="p">,</span> <span class="mi">0</span><span class="p">)</span> <span class="o">+</span> <span class="n">n</span></span><a href="#l163"></a>
<span id="l164">    <span class="c"># Find the expected result type. If allowed_types has only one item, it</span></span><a href="#l164"></a>
<span id="l165">    <span class="c"># will be int; if it has two, use the one which isn&#39;t int.</span></span><a href="#l165"></a>
<span id="l166">    <span class="k">assert</span> <span class="nb">len</span><span class="p">(</span><span class="n">allowed_types</span><span class="p">)</span> <span class="ow">in</span> <span class="p">(</span><span class="mi">1</span><span class="p">,</span> <span class="mi">2</span><span class="p">)</span></span><a href="#l166"></a>
<span id="l167">    <span class="k">if</span> <span class="nb">len</span><span class="p">(</span><span class="n">allowed_types</span><span class="p">)</span> <span class="o">==</span> <span class="mi">1</span><span class="p">:</span></span><a href="#l167"></a>
<span id="l168">        <span class="k">assert</span> <span class="n">allowed_types</span><span class="o">.</span><span class="n">pop</span><span class="p">()</span> <span class="ow">is</span> <span class="nb">int</span></span><a href="#l168"></a>
<span id="l169">        <span class="n">T</span> <span class="o">=</span> <span class="nb">int</span></span><a href="#l169"></a>
<span id="l170">    <span class="k">else</span><span class="p">:</span></span><a href="#l170"></a>
<span id="l171">        <span class="n">T</span> <span class="o">=</span> <span class="p">(</span><span class="n">allowed_types</span> <span class="o">-</span> <span class="nb">set</span><span class="p">([</span><span class="nb">int</span><span class="p">]))</span><span class="o">.</span><span class="n">pop</span><span class="p">()</span></span><a href="#l171"></a>
<span id="l172">    <span class="k">if</span> <span class="bp">None</span> <span class="ow">in</span> <span class="n">partials</span><span class="p">:</span></span><a href="#l172"></a>
<span id="l173">        <span class="k">assert</span> <span class="nb">issubclass</span><span class="p">(</span><span class="n">T</span><span class="p">,</span> <span class="p">(</span><span class="nb">float</span><span class="p">,</span> <span class="n">Decimal</span><span class="p">))</span></span><a href="#l173"></a>
<span id="l174">        <span class="k">assert</span> <span class="ow">not</span> <span class="n">math</span><span class="o">.</span><span class="n">isfinite</span><span class="p">(</span><span class="n">partials</span><span class="p">[</span><span class="bp">None</span><span class="p">])</span></span><a href="#l174"></a>
<span id="l175">        <span class="k">return</span> <span class="n">T</span><span class="p">(</span><span class="n">partials</span><span class="p">[</span><span class="bp">None</span><span class="p">])</span></span><a href="#l175"></a>
<span id="l176">    <span class="n">total</span> <span class="o">=</span> <span class="n">Fraction</span><span class="p">()</span></span><a href="#l176"></a>
<span id="l177">    <span class="k">for</span> <span class="n">d</span><span class="p">,</span> <span class="n">n</span> <span class="ow">in</span> <span class="nb">sorted</span><span class="p">(</span><span class="n">partials</span><span class="o">.</span><span class="n">items</span><span class="p">()):</span></span><a href="#l177"></a>
<span id="l178">        <span class="n">total</span> <span class="o">+=</span> <span class="n">Fraction</span><span class="p">(</span><span class="n">n</span><span class="p">,</span> <span class="n">d</span><span class="p">)</span></span><a href="#l178"></a>
<span id="l179">    <span class="k">if</span> <span class="nb">issubclass</span><span class="p">(</span><span class="n">T</span><span class="p">,</span> <span class="nb">int</span><span class="p">):</span></span><a href="#l179"></a>
<span id="l180">        <span class="k">assert</span> <span class="n">total</span><span class="o">.</span><span class="n">denominator</span> <span class="o">==</span> <span class="mi">1</span></span><a href="#l180"></a>
<span id="l181">        <span class="k">return</span> <span class="n">T</span><span class="p">(</span><span class="n">total</span><span class="o">.</span><span class="n">numerator</span><span class="p">)</span></span><a href="#l181"></a>
<span id="l182">    <span class="k">if</span> <span class="nb">issubclass</span><span class="p">(</span><span class="n">T</span><span class="p">,</span> <span class="n">Decimal</span><span class="p">):</span></span><a href="#l182"></a>
<span id="l183">        <span class="k">return</span> <span class="n">T</span><span class="p">(</span><span class="n">total</span><span class="o">.</span><span class="n">numerator</span><span class="p">)</span><span class="o">/</span><span class="n">total</span><span class="o">.</span><span class="n">denominator</span></span><a href="#l183"></a>
<span id="l184">    <span class="k">return</span> <span class="n">T</span><span class="p">(</span><span class="n">total</span><span class="p">)</span></span><a href="#l184"></a>
<span id="l185"></span><a href="#l185"></a>
<span id="l186"></span><a href="#l186"></a>
<span id="l187"><span class="k">def</span> <span class="nf">_check_type</span><span class="p">(</span><span class="n">T</span><span class="p">,</span> <span class="n">allowed</span><span class="p">):</span></span><a href="#l187"></a>
<span id="l188">    <span class="k">if</span> <span class="n">T</span> <span class="ow">not</span> <span class="ow">in</span> <span class="n">allowed</span><span class="p">:</span></span><a href="#l188"></a>
<span id="l189">        <span class="k">if</span> <span class="nb">len</span><span class="p">(</span><span class="n">allowed</span><span class="p">)</span> <span class="o">==</span> <span class="mi">1</span><span class="p">:</span></span><a href="#l189"></a>
<span id="l190">            <span class="n">allowed</span><span class="o">.</span><span class="n">add</span><span class="p">(</span><span class="n">T</span><span class="p">)</span></span><a href="#l190"></a>
<span id="l191">        <span class="k">else</span><span class="p">:</span></span><a href="#l191"></a>
<span id="l192">            <span class="n">types</span> <span class="o">=</span> <span class="s">&#39;, &#39;</span><span class="o">.</span><span class="n">join</span><span class="p">([</span><span class="n">t</span><span class="o">.</span><span class="n">__name__</span> <span class="k">for</span> <span class="n">t</span> <span class="ow">in</span> <span class="n">allowed</span><span class="p">]</span> <span class="o">+</span> <span class="p">[</span><span class="n">T</span><span class="o">.</span><span class="n">__name__</span><span class="p">])</span></span><a href="#l192"></a>
<span id="l193">            <span class="k">raise</span> <span class="ne">TypeError</span><span class="p">(</span><span class="s">&quot;unsupported mixed types: </span><span class="si">%s</span><span class="s">&quot;</span> <span class="o">%</span> <span class="n">types</span><span class="p">)</span></span><a href="#l193"></a>
<span id="l194"></span><a href="#l194"></a>
<span id="l195"></span><a href="#l195"></a>
<span id="l196"><span class="k">def</span> <span class="nf">_exact_ratio</span><span class="p">(</span><span class="n">x</span><span class="p">):</span></span><a href="#l196"></a>
<span id="l197">    <span class="sd">&quot;&quot;&quot;Convert Real number x exactly to (numerator, denominator) pair.</span></span><a href="#l197"></a>
<span id="l198"></span><a href="#l198"></a>
<span id="l199"><span class="sd">    &gt;&gt;&gt; _exact_ratio(0.25)</span></span><a href="#l199"></a>
<span id="l200"><span class="sd">    (1, 4)</span></span><a href="#l200"></a>
<span id="l201"></span><a href="#l201"></a>
<span id="l202"><span class="sd">    x is expected to be an int, Fraction, Decimal or float.</span></span><a href="#l202"></a>
<span id="l203"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l203"></a>
<span id="l204">    <span class="k">try</span><span class="p">:</span></span><a href="#l204"></a>
<span id="l205">        <span class="k">try</span><span class="p">:</span></span><a href="#l205"></a>
<span id="l206">            <span class="c"># int, Fraction</span></span><a href="#l206"></a>
<span id="l207">            <span class="k">return</span> <span class="p">(</span><span class="n">x</span><span class="o">.</span><span class="n">numerator</span><span class="p">,</span> <span class="n">x</span><span class="o">.</span><span class="n">denominator</span><span class="p">)</span></span><a href="#l207"></a>
<span id="l208">        <span class="k">except</span> <span class="ne">AttributeError</span><span class="p">:</span></span><a href="#l208"></a>
<span id="l209">            <span class="c"># float</span></span><a href="#l209"></a>
<span id="l210">            <span class="k">try</span><span class="p">:</span></span><a href="#l210"></a>
<span id="l211">                <span class="k">return</span> <span class="n">x</span><span class="o">.</span><span class="n">as_integer_ratio</span><span class="p">()</span></span><a href="#l211"></a>
<span id="l212">            <span class="k">except</span> <span class="ne">AttributeError</span><span class="p">:</span></span><a href="#l212"></a>
<span id="l213">                <span class="c"># Decimal</span></span><a href="#l213"></a>
<span id="l214">                <span class="k">try</span><span class="p">:</span></span><a href="#l214"></a>
<span id="l215">                    <span class="k">return</span> <span class="n">_decimal_to_ratio</span><span class="p">(</span><span class="n">x</span><span class="p">)</span></span><a href="#l215"></a>
<span id="l216">                <span class="k">except</span> <span class="ne">AttributeError</span><span class="p">:</span></span><a href="#l216"></a>
<span id="l217">                    <span class="n">msg</span> <span class="o">=</span> <span class="s">&quot;can&#39;t convert type &#39;{}&#39; to numerator/denominator&quot;</span></span><a href="#l217"></a>
<span id="l218">                    <span class="k">raise</span> <span class="ne">TypeError</span><span class="p">(</span><span class="n">msg</span><span class="o">.</span><span class="n">format</span><span class="p">(</span><span class="nb">type</span><span class="p">(</span><span class="n">x</span><span class="p">)</span><span class="o">.</span><span class="n">__name__</span><span class="p">))</span> <span class="kn">from</span> <span class="bp">None</span></span><a href="#l218"></a>
<span id="l219">    <span class="k">except</span> <span class="p">(</span><span class="ne">OverflowError</span><span class="p">,</span> <span class="ne">ValueError</span><span class="p">):</span></span><a href="#l219"></a>
<span id="l220">        <span class="c"># INF or NAN</span></span><a href="#l220"></a>
<span id="l221">        <span class="k">if</span> <span class="n">__debug__</span><span class="p">:</span></span><a href="#l221"></a>
<span id="l222">            <span class="c"># Decimal signalling NANs cannot be converted to float :-(</span></span><a href="#l222"></a>
<span id="l223">            <span class="k">if</span> <span class="nb">isinstance</span><span class="p">(</span><span class="n">x</span><span class="p">,</span> <span class="n">Decimal</span><span class="p">):</span></span><a href="#l223"></a>
<span id="l224">                <span class="k">assert</span> <span class="ow">not</span> <span class="n">x</span><span class="o">.</span><span class="n">is_finite</span><span class="p">()</span></span><a href="#l224"></a>
<span id="l225">            <span class="k">else</span><span class="p">:</span></span><a href="#l225"></a>
<span id="l226">                <span class="k">assert</span> <span class="ow">not</span> <span class="n">math</span><span class="o">.</span><span class="n">isfinite</span><span class="p">(</span><span class="n">x</span><span class="p">)</span></span><a href="#l226"></a>
<span id="l227">        <span class="k">return</span> <span class="p">(</span><span class="n">x</span><span class="p">,</span> <span class="bp">None</span><span class="p">)</span></span><a href="#l227"></a>
<span id="l228"></span><a href="#l228"></a>
<span id="l229"></span><a href="#l229"></a>
<span id="l230"><span class="c"># FIXME This is faster than Fraction.from_decimal, but still too slow.</span></span><a href="#l230"></a>
<span id="l231"><span class="k">def</span> <span class="nf">_decimal_to_ratio</span><span class="p">(</span><span class="n">d</span><span class="p">):</span></span><a href="#l231"></a>
<span id="l232">    <span class="sd">&quot;&quot;&quot;Convert Decimal d to exact integer ratio (numerator, denominator).</span></span><a href="#l232"></a>
<span id="l233"></span><a href="#l233"></a>
<span id="l234"><span class="sd">    &gt;&gt;&gt; from decimal import Decimal</span></span><a href="#l234"></a>
<span id="l235"><span class="sd">    &gt;&gt;&gt; _decimal_to_ratio(Decimal(&quot;2.6&quot;))</span></span><a href="#l235"></a>
<span id="l236"><span class="sd">    (26, 10)</span></span><a href="#l236"></a>
<span id="l237"></span><a href="#l237"></a>
<span id="l238"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l238"></a>
<span id="l239">    <span class="n">sign</span><span class="p">,</span> <span class="n">digits</span><span class="p">,</span> <span class="n">exp</span> <span class="o">=</span> <span class="n">d</span><span class="o">.</span><span class="n">as_tuple</span><span class="p">()</span></span><a href="#l239"></a>
<span id="l240">    <span class="k">if</span> <span class="n">exp</span> <span class="ow">in</span> <span class="p">(</span><span class="s">&#39;F&#39;</span><span class="p">,</span> <span class="s">&#39;n&#39;</span><span class="p">,</span> <span class="s">&#39;N&#39;</span><span class="p">):</span>  <span class="c"># INF, NAN, sNAN</span></span><a href="#l240"></a>
<span id="l241">        <span class="k">assert</span> <span class="ow">not</span> <span class="n">d</span><span class="o">.</span><span class="n">is_finite</span><span class="p">()</span></span><a href="#l241"></a>
<span id="l242">        <span class="k">raise</span> <span class="ne">ValueError</span></span><a href="#l242"></a>
<span id="l243">    <span class="n">num</span> <span class="o">=</span> <span class="mi">0</span></span><a href="#l243"></a>
<span id="l244">    <span class="k">for</span> <span class="n">digit</span> <span class="ow">in</span> <span class="n">digits</span><span class="p">:</span></span><a href="#l244"></a>
<span id="l245">        <span class="n">num</span> <span class="o">=</span> <span class="n">num</span><span class="o">*</span><span class="mi">10</span> <span class="o">+</span> <span class="n">digit</span></span><a href="#l245"></a>
<span id="l246">    <span class="k">if</span> <span class="n">exp</span> <span class="o">&lt;</span> <span class="mi">0</span><span class="p">:</span></span><a href="#l246"></a>
<span id="l247">        <span class="n">den</span> <span class="o">=</span> <span class="mi">10</span><span class="o">**-</span><span class="n">exp</span></span><a href="#l247"></a>
<span id="l248">    <span class="k">else</span><span class="p">:</span></span><a href="#l248"></a>
<span id="l249">        <span class="n">num</span> <span class="o">*=</span> <span class="mi">10</span><span class="o">**</span><span class="n">exp</span></span><a href="#l249"></a>
<span id="l250">        <span class="n">den</span> <span class="o">=</span> <span class="mi">1</span></span><a href="#l250"></a>
<span id="l251">    <span class="k">if</span> <span class="n">sign</span><span class="p">:</span></span><a href="#l251"></a>
<span id="l252">        <span class="n">num</span> <span class="o">=</span> <span class="o">-</span><span class="n">num</span></span><a href="#l252"></a>
<span id="l253">    <span class="k">return</span> <span class="p">(</span><span class="n">num</span><span class="p">,</span> <span class="n">den</span><span class="p">)</span></span><a href="#l253"></a>
<span id="l254"></span><a href="#l254"></a>
<span id="l255"></span><a href="#l255"></a>
<span id="l256"><span class="k">def</span> <span class="nf">_counts</span><span class="p">(</span><span class="n">data</span><span class="p">):</span></span><a href="#l256"></a>
<span id="l257">    <span class="c"># Generate a table of sorted (value, frequency) pairs.</span></span><a href="#l257"></a>
<span id="l258">    <span class="n">table</span> <span class="o">=</span> <span class="n">collections</span><span class="o">.</span><span class="n">Counter</span><span class="p">(</span><span class="nb">iter</span><span class="p">(</span><span class="n">data</span><span class="p">))</span><span class="o">.</span><span class="n">most_common</span><span class="p">()</span></span><a href="#l258"></a>
<span id="l259">    <span class="k">if</span> <span class="ow">not</span> <span class="n">table</span><span class="p">:</span></span><a href="#l259"></a>
<span id="l260">        <span class="k">return</span> <span class="n">table</span></span><a href="#l260"></a>
<span id="l261">    <span class="c"># Extract the values with the highest frequency.</span></span><a href="#l261"></a>
<span id="l262">    <span class="n">maxfreq</span> <span class="o">=</span> <span class="n">table</span><span class="p">[</span><span class="mi">0</span><span class="p">][</span><span class="mi">1</span><span class="p">]</span></span><a href="#l262"></a>
<span id="l263">    <span class="k">for</span> <span class="n">i</span> <span class="ow">in</span> <span class="nb">range</span><span class="p">(</span><span class="mi">1</span><span class="p">,</span> <span class="nb">len</span><span class="p">(</span><span class="n">table</span><span class="p">)):</span></span><a href="#l263"></a>
<span id="l264">        <span class="k">if</span> <span class="n">table</span><span class="p">[</span><span class="n">i</span><span class="p">][</span><span class="mi">1</span><span class="p">]</span> <span class="o">!=</span> <span class="n">maxfreq</span><span class="p">:</span></span><a href="#l264"></a>
<span id="l265">            <span class="n">table</span> <span class="o">=</span> <span class="n">table</span><span class="p">[:</span><span class="n">i</span><span class="p">]</span></span><a href="#l265"></a>
<span id="l266">            <span class="k">break</span></span><a href="#l266"></a>
<span id="l267">    <span class="k">return</span> <span class="n">table</span></span><a href="#l267"></a>
<span id="l268"></span><a href="#l268"></a>
<span id="l269"></span><a href="#l269"></a>
<span id="l270"><span class="c"># === Measures of central tendency (averages) ===</span></span><a href="#l270"></a>
<span id="l271"></span><a href="#l271"></a>
<span id="l272"><span class="k">def</span> <span class="nf">mean</span><span class="p">(</span><span class="n">data</span><span class="p">):</span></span><a href="#l272"></a>
<span id="l273">    <span class="sd">&quot;&quot;&quot;Return the sample arithmetic mean of data.</span></span><a href="#l273"></a>
<span id="l274"></span><a href="#l274"></a>
<span id="l275"><span class="sd">    &gt;&gt;&gt; mean([1, 2, 3, 4, 4])</span></span><a href="#l275"></a>
<span id="l276"><span class="sd">    2.8</span></span><a href="#l276"></a>
<span id="l277"></span><a href="#l277"></a>
<span id="l278"><span class="sd">    &gt;&gt;&gt; from fractions import Fraction as F</span></span><a href="#l278"></a>
<span id="l279"><span class="sd">    &gt;&gt;&gt; mean([F(3, 7), F(1, 21), F(5, 3), F(1, 3)])</span></span><a href="#l279"></a>
<span id="l280"><span class="sd">    Fraction(13, 21)</span></span><a href="#l280"></a>
<span id="l281"></span><a href="#l281"></a>
<span id="l282"><span class="sd">    &gt;&gt;&gt; from decimal import Decimal as D</span></span><a href="#l282"></a>
<span id="l283"><span class="sd">    &gt;&gt;&gt; mean([D(&quot;0.5&quot;), D(&quot;0.75&quot;), D(&quot;0.625&quot;), D(&quot;0.375&quot;)])</span></span><a href="#l283"></a>
<span id="l284"><span class="sd">    Decimal(&#39;0.5625&#39;)</span></span><a href="#l284"></a>
<span id="l285"></span><a href="#l285"></a>
<span id="l286"><span class="sd">    If ``data`` is empty, StatisticsError will be raised.</span></span><a href="#l286"></a>
<span id="l287"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l287"></a>
<span id="l288">    <span class="k">if</span> <span class="nb">iter</span><span class="p">(</span><span class="n">data</span><span class="p">)</span> <span class="ow">is</span> <span class="n">data</span><span class="p">:</span></span><a href="#l288"></a>
<span id="l289">        <span class="n">data</span> <span class="o">=</span> <span class="nb">list</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l289"></a>
<span id="l290">    <span class="n">n</span> <span class="o">=</span> <span class="nb">len</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l290"></a>
<span id="l291">    <span class="k">if</span> <span class="n">n</span> <span class="o">&lt;</span> <span class="mi">1</span><span class="p">:</span></span><a href="#l291"></a>
<span id="l292">        <span class="k">raise</span> <span class="n">StatisticsError</span><span class="p">(</span><span class="s">&#39;mean requires at least one data point&#39;</span><span class="p">)</span></span><a href="#l292"></a>
<span id="l293">    <span class="k">return</span> <span class="n">_sum</span><span class="p">(</span><span class="n">data</span><span class="p">)</span><span class="o">/</span><span class="n">n</span></span><a href="#l293"></a>
<span id="l294"></span><a href="#l294"></a>
<span id="l295"></span><a href="#l295"></a>
<span id="l296"><span class="c"># FIXME: investigate ways to calculate medians without sorting? Quickselect?</span></span><a href="#l296"></a>
<span id="l297"><span class="k">def</span> <span class="nf">median</span><span class="p">(</span><span class="n">data</span><span class="p">):</span></span><a href="#l297"></a>
<span id="l298">    <span class="sd">&quot;&quot;&quot;Return the median (middle value) of numeric data.</span></span><a href="#l298"></a>
<span id="l299"></span><a href="#l299"></a>
<span id="l300"><span class="sd">    When the number of data points is odd, return the middle data point.</span></span><a href="#l300"></a>
<span id="l301"><span class="sd">    When the number of data points is even, the median is interpolated by</span></span><a href="#l301"></a>
<span id="l302"><span class="sd">    taking the average of the two middle values:</span></span><a href="#l302"></a>
<span id="l303"></span><a href="#l303"></a>
<span id="l304"><span class="sd">    &gt;&gt;&gt; median([1, 3, 5])</span></span><a href="#l304"></a>
<span id="l305"><span class="sd">    3</span></span><a href="#l305"></a>
<span id="l306"><span class="sd">    &gt;&gt;&gt; median([1, 3, 5, 7])</span></span><a href="#l306"></a>
<span id="l307"><span class="sd">    4.0</span></span><a href="#l307"></a>
<span id="l308"></span><a href="#l308"></a>
<span id="l309"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l309"></a>
<span id="l310">    <span class="n">data</span> <span class="o">=</span> <span class="nb">sorted</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l310"></a>
<span id="l311">    <span class="n">n</span> <span class="o">=</span> <span class="nb">len</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l311"></a>
<span id="l312">    <span class="k">if</span> <span class="n">n</span> <span class="o">==</span> <span class="mi">0</span><span class="p">:</span></span><a href="#l312"></a>
<span id="l313">        <span class="k">raise</span> <span class="n">StatisticsError</span><span class="p">(</span><span class="s">&quot;no median for empty data&quot;</span><span class="p">)</span></span><a href="#l313"></a>
<span id="l314">    <span class="k">if</span> <span class="n">n</span><span class="o">%</span><span class="mi">2</span> <span class="o">==</span> <span class="mi">1</span><span class="p">:</span></span><a href="#l314"></a>
<span id="l315">        <span class="k">return</span> <span class="n">data</span><span class="p">[</span><span class="n">n</span><span class="o">//</span><span class="mi">2</span><span class="p">]</span></span><a href="#l315"></a>
<span id="l316">    <span class="k">else</span><span class="p">:</span></span><a href="#l316"></a>
<span id="l317">        <span class="n">i</span> <span class="o">=</span> <span class="n">n</span><span class="o">//</span><span class="mi">2</span></span><a href="#l317"></a>
<span id="l318">        <span class="k">return</span> <span class="p">(</span><span class="n">data</span><span class="p">[</span><span class="n">i</span> <span class="o">-</span> <span class="mi">1</span><span class="p">]</span> <span class="o">+</span> <span class="n">data</span><span class="p">[</span><span class="n">i</span><span class="p">])</span><span class="o">/</span><span class="mi">2</span></span><a href="#l318"></a>
<span id="l319"></span><a href="#l319"></a>
<span id="l320"></span><a href="#l320"></a>
<span id="l321"><span class="k">def</span> <span class="nf">median_low</span><span class="p">(</span><span class="n">data</span><span class="p">):</span></span><a href="#l321"></a>
<span id="l322">    <span class="sd">&quot;&quot;&quot;Return the low median of numeric data.</span></span><a href="#l322"></a>
<span id="l323"></span><a href="#l323"></a>
<span id="l324"><span class="sd">    When the number of data points is odd, the middle value is returned.</span></span><a href="#l324"></a>
<span id="l325"><span class="sd">    When it is even, the smaller of the two middle values is returned.</span></span><a href="#l325"></a>
<span id="l326"></span><a href="#l326"></a>
<span id="l327"><span class="sd">    &gt;&gt;&gt; median_low([1, 3, 5])</span></span><a href="#l327"></a>
<span id="l328"><span class="sd">    3</span></span><a href="#l328"></a>
<span id="l329"><span class="sd">    &gt;&gt;&gt; median_low([1, 3, 5, 7])</span></span><a href="#l329"></a>
<span id="l330"><span class="sd">    3</span></span><a href="#l330"></a>
<span id="l331"></span><a href="#l331"></a>
<span id="l332"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l332"></a>
<span id="l333">    <span class="n">data</span> <span class="o">=</span> <span class="nb">sorted</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l333"></a>
<span id="l334">    <span class="n">n</span> <span class="o">=</span> <span class="nb">len</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l334"></a>
<span id="l335">    <span class="k">if</span> <span class="n">n</span> <span class="o">==</span> <span class="mi">0</span><span class="p">:</span></span><a href="#l335"></a>
<span id="l336">        <span class="k">raise</span> <span class="n">StatisticsError</span><span class="p">(</span><span class="s">&quot;no median for empty data&quot;</span><span class="p">)</span></span><a href="#l336"></a>
<span id="l337">    <span class="k">if</span> <span class="n">n</span><span class="o">%</span><span class="mi">2</span> <span class="o">==</span> <span class="mi">1</span><span class="p">:</span></span><a href="#l337"></a>
<span id="l338">        <span class="k">return</span> <span class="n">data</span><span class="p">[</span><span class="n">n</span><span class="o">//</span><span class="mi">2</span><span class="p">]</span></span><a href="#l338"></a>
<span id="l339">    <span class="k">else</span><span class="p">:</span></span><a href="#l339"></a>
<span id="l340">        <span class="k">return</span> <span class="n">data</span><span class="p">[</span><span class="n">n</span><span class="o">//</span><span class="mi">2</span> <span class="o">-</span> <span class="mi">1</span><span class="p">]</span></span><a href="#l340"></a>
<span id="l341"></span><a href="#l341"></a>
<span id="l342"></span><a href="#l342"></a>
<span id="l343"><span class="k">def</span> <span class="nf">median_high</span><span class="p">(</span><span class="n">data</span><span class="p">):</span></span><a href="#l343"></a>
<span id="l344">    <span class="sd">&quot;&quot;&quot;Return the high median of data.</span></span><a href="#l344"></a>
<span id="l345"></span><a href="#l345"></a>
<span id="l346"><span class="sd">    When the number of data points is odd, the middle value is returned.</span></span><a href="#l346"></a>
<span id="l347"><span class="sd">    When it is even, the larger of the two middle values is returned.</span></span><a href="#l347"></a>
<span id="l348"></span><a href="#l348"></a>
<span id="l349"><span class="sd">    &gt;&gt;&gt; median_high([1, 3, 5])</span></span><a href="#l349"></a>
<span id="l350"><span class="sd">    3</span></span><a href="#l350"></a>
<span id="l351"><span class="sd">    &gt;&gt;&gt; median_high([1, 3, 5, 7])</span></span><a href="#l351"></a>
<span id="l352"><span class="sd">    5</span></span><a href="#l352"></a>
<span id="l353"></span><a href="#l353"></a>
<span id="l354"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l354"></a>
<span id="l355">    <span class="n">data</span> <span class="o">=</span> <span class="nb">sorted</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l355"></a>
<span id="l356">    <span class="n">n</span> <span class="o">=</span> <span class="nb">len</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l356"></a>
<span id="l357">    <span class="k">if</span> <span class="n">n</span> <span class="o">==</span> <span class="mi">0</span><span class="p">:</span></span><a href="#l357"></a>
<span id="l358">        <span class="k">raise</span> <span class="n">StatisticsError</span><span class="p">(</span><span class="s">&quot;no median for empty data&quot;</span><span class="p">)</span></span><a href="#l358"></a>
<span id="l359">    <span class="k">return</span> <span class="n">data</span><span class="p">[</span><span class="n">n</span><span class="o">//</span><span class="mi">2</span><span class="p">]</span></span><a href="#l359"></a>
<span id="l360"></span><a href="#l360"></a>
<span id="l361"></span><a href="#l361"></a>
<span id="l362"><span class="k">def</span> <span class="nf">median_grouped</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">interval</span><span class="o">=</span><span class="mi">1</span><span class="p">):</span></span><a href="#l362"></a>
<span id="l363">    <span class="sd">&quot;&quot;&quot;&quot;Return the 50th percentile (median) of grouped continuous data.</span></span><a href="#l363"></a>
<span id="l364"></span><a href="#l364"></a>
<span id="l365"><span class="sd">    &gt;&gt;&gt; median_grouped([1, 2, 2, 3, 4, 4, 4, 4, 4, 5])</span></span><a href="#l365"></a>
<span id="l366"><span class="sd">    3.7</span></span><a href="#l366"></a>
<span id="l367"><span class="sd">    &gt;&gt;&gt; median_grouped([52, 52, 53, 54])</span></span><a href="#l367"></a>
<span id="l368"><span class="sd">    52.5</span></span><a href="#l368"></a>
<span id="l369"></span><a href="#l369"></a>
<span id="l370"><span class="sd">    This calculates the median as the 50th percentile, and should be</span></span><a href="#l370"></a>
<span id="l371"><span class="sd">    used when your data is continuous and grouped. In the above example,</span></span><a href="#l371"></a>
<span id="l372"><span class="sd">    the values 1, 2, 3, etc. actually represent the midpoint of classes</span></span><a href="#l372"></a>
<span id="l373"><span class="sd">    0.5-1.5, 1.5-2.5, 2.5-3.5, etc. The middle value falls somewhere in</span></span><a href="#l373"></a>
<span id="l374"><span class="sd">    class 3.5-4.5, and interpolation is used to estimate it.</span></span><a href="#l374"></a>
<span id="l375"></span><a href="#l375"></a>
<span id="l376"><span class="sd">    Optional argument ``interval`` represents the class interval, and</span></span><a href="#l376"></a>
<span id="l377"><span class="sd">    defaults to 1. Changing the class interval naturally will change the</span></span><a href="#l377"></a>
<span id="l378"><span class="sd">    interpolated 50th percentile value:</span></span><a href="#l378"></a>
<span id="l379"></span><a href="#l379"></a>
<span id="l380"><span class="sd">    &gt;&gt;&gt; median_grouped([1, 3, 3, 5, 7], interval=1)</span></span><a href="#l380"></a>
<span id="l381"><span class="sd">    3.25</span></span><a href="#l381"></a>
<span id="l382"><span class="sd">    &gt;&gt;&gt; median_grouped([1, 3, 3, 5, 7], interval=2)</span></span><a href="#l382"></a>
<span id="l383"><span class="sd">    3.5</span></span><a href="#l383"></a>
<span id="l384"></span><a href="#l384"></a>
<span id="l385"><span class="sd">    This function does not check whether the data points are at least</span></span><a href="#l385"></a>
<span id="l386"><span class="sd">    ``interval`` apart.</span></span><a href="#l386"></a>
<span id="l387"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l387"></a>
<span id="l388">    <span class="n">data</span> <span class="o">=</span> <span class="nb">sorted</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l388"></a>
<span id="l389">    <span class="n">n</span> <span class="o">=</span> <span class="nb">len</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l389"></a>
<span id="l390">    <span class="k">if</span> <span class="n">n</span> <span class="o">==</span> <span class="mi">0</span><span class="p">:</span></span><a href="#l390"></a>
<span id="l391">        <span class="k">raise</span> <span class="n">StatisticsError</span><span class="p">(</span><span class="s">&quot;no median for empty data&quot;</span><span class="p">)</span></span><a href="#l391"></a>
<span id="l392">    <span class="k">elif</span> <span class="n">n</span> <span class="o">==</span> <span class="mi">1</span><span class="p">:</span></span><a href="#l392"></a>
<span id="l393">        <span class="k">return</span> <span class="n">data</span><span class="p">[</span><span class="mi">0</span><span class="p">]</span></span><a href="#l393"></a>
<span id="l394">    <span class="c"># Find the value at the midpoint. Remember this corresponds to the</span></span><a href="#l394"></a>
<span id="l395">    <span class="c"># centre of the class interval.</span></span><a href="#l395"></a>
<span id="l396">    <span class="n">x</span> <span class="o">=</span> <span class="n">data</span><span class="p">[</span><span class="n">n</span><span class="o">//</span><span class="mi">2</span><span class="p">]</span></span><a href="#l396"></a>
<span id="l397">    <span class="k">for</span> <span class="n">obj</span> <span class="ow">in</span> <span class="p">(</span><span class="n">x</span><span class="p">,</span> <span class="n">interval</span><span class="p">):</span></span><a href="#l397"></a>
<span id="l398">        <span class="k">if</span> <span class="nb">isinstance</span><span class="p">(</span><span class="n">obj</span><span class="p">,</span> <span class="p">(</span><span class="nb">str</span><span class="p">,</span> <span class="nb">bytes</span><span class="p">)):</span></span><a href="#l398"></a>
<span id="l399">            <span class="k">raise</span> <span class="ne">TypeError</span><span class="p">(</span><span class="s">&#39;expected number but got </span><span class="si">%r</span><span class="s">&#39;</span> <span class="o">%</span> <span class="n">obj</span><span class="p">)</span></span><a href="#l399"></a>
<span id="l400">    <span class="k">try</span><span class="p">:</span></span><a href="#l400"></a>
<span id="l401">        <span class="n">L</span> <span class="o">=</span> <span class="n">x</span> <span class="o">-</span> <span class="n">interval</span><span class="o">/</span><span class="mi">2</span>  <span class="c"># The lower limit of the median interval.</span></span><a href="#l401"></a>
<span id="l402">    <span class="k">except</span> <span class="ne">TypeError</span><span class="p">:</span></span><a href="#l402"></a>
<span id="l403">        <span class="c"># Mixed type. For now we just coerce to float.</span></span><a href="#l403"></a>
<span id="l404">        <span class="n">L</span> <span class="o">=</span> <span class="nb">float</span><span class="p">(</span><span class="n">x</span><span class="p">)</span> <span class="o">-</span> <span class="nb">float</span><span class="p">(</span><span class="n">interval</span><span class="p">)</span><span class="o">/</span><span class="mi">2</span></span><a href="#l404"></a>
<span id="l405">    <span class="n">cf</span> <span class="o">=</span> <span class="n">data</span><span class="o">.</span><span class="n">index</span><span class="p">(</span><span class="n">x</span><span class="p">)</span>  <span class="c"># Number of values below the median interval.</span></span><a href="#l405"></a>
<span id="l406">    <span class="c"># FIXME The following line could be more efficient for big lists.</span></span><a href="#l406"></a>
<span id="l407">    <span class="n">f</span> <span class="o">=</span> <span class="n">data</span><span class="o">.</span><span class="n">count</span><span class="p">(</span><span class="n">x</span><span class="p">)</span>  <span class="c"># Number of data points in the median interval.</span></span><a href="#l407"></a>
<span id="l408">    <span class="k">return</span> <span class="n">L</span> <span class="o">+</span> <span class="n">interval</span><span class="o">*</span><span class="p">(</span><span class="n">n</span><span class="o">/</span><span class="mi">2</span> <span class="o">-</span> <span class="n">cf</span><span class="p">)</span><span class="o">/</span><span class="n">f</span></span><a href="#l408"></a>
<span id="l409"></span><a href="#l409"></a>
<span id="l410"></span><a href="#l410"></a>
<span id="l411"><span class="k">def</span> <span class="nf">mode</span><span class="p">(</span><span class="n">data</span><span class="p">):</span></span><a href="#l411"></a>
<span id="l412">    <span class="sd">&quot;&quot;&quot;Return the most common data point from discrete or nominal data.</span></span><a href="#l412"></a>
<span id="l413"></span><a href="#l413"></a>
<span id="l414"><span class="sd">    ``mode`` assumes discrete data, and returns a single value. This is the</span></span><a href="#l414"></a>
<span id="l415"><span class="sd">    standard treatment of the mode as commonly taught in schools:</span></span><a href="#l415"></a>
<span id="l416"></span><a href="#l416"></a>
<span id="l417"><span class="sd">    &gt;&gt;&gt; mode([1, 1, 2, 3, 3, 3, 3, 4])</span></span><a href="#l417"></a>
<span id="l418"><span class="sd">    3</span></span><a href="#l418"></a>
<span id="l419"></span><a href="#l419"></a>
<span id="l420"><span class="sd">    This also works with nominal (non-numeric) data:</span></span><a href="#l420"></a>
<span id="l421"></span><a href="#l421"></a>
<span id="l422"><span class="sd">    &gt;&gt;&gt; mode([&quot;red&quot;, &quot;blue&quot;, &quot;blue&quot;, &quot;red&quot;, &quot;green&quot;, &quot;red&quot;, &quot;red&quot;])</span></span><a href="#l422"></a>
<span id="l423"><span class="sd">    &#39;red&#39;</span></span><a href="#l423"></a>
<span id="l424"></span><a href="#l424"></a>
<span id="l425"><span class="sd">    If there is not exactly one most common value, ``mode`` will raise</span></span><a href="#l425"></a>
<span id="l426"><span class="sd">    StatisticsError.</span></span><a href="#l426"></a>
<span id="l427"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l427"></a>
<span id="l428">    <span class="c"># Generate a table of sorted (value, frequency) pairs.</span></span><a href="#l428"></a>
<span id="l429">    <span class="n">table</span> <span class="o">=</span> <span class="n">_counts</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l429"></a>
<span id="l430">    <span class="k">if</span> <span class="nb">len</span><span class="p">(</span><span class="n">table</span><span class="p">)</span> <span class="o">==</span> <span class="mi">1</span><span class="p">:</span></span><a href="#l430"></a>
<span id="l431">        <span class="k">return</span> <span class="n">table</span><span class="p">[</span><span class="mi">0</span><span class="p">][</span><span class="mi">0</span><span class="p">]</span></span><a href="#l431"></a>
<span id="l432">    <span class="k">elif</span> <span class="n">table</span><span class="p">:</span></span><a href="#l432"></a>
<span id="l433">        <span class="k">raise</span> <span class="n">StatisticsError</span><span class="p">(</span></span><a href="#l433"></a>
<span id="l434">                <span class="s">&#39;no unique mode; found </span><span class="si">%d</span><span class="s"> equally common values&#39;</span> <span class="o">%</span> <span class="nb">len</span><span class="p">(</span><span class="n">table</span><span class="p">)</span></span><a href="#l434"></a>
<span id="l435">                <span class="p">)</span></span><a href="#l435"></a>
<span id="l436">    <span class="k">else</span><span class="p">:</span></span><a href="#l436"></a>
<span id="l437">        <span class="k">raise</span> <span class="n">StatisticsError</span><span class="p">(</span><span class="s">&#39;no mode for empty data&#39;</span><span class="p">)</span></span><a href="#l437"></a>
<span id="l438"></span><a href="#l438"></a>
<span id="l439"></span><a href="#l439"></a>
<span id="l440"><span class="c"># === Measures of spread ===</span></span><a href="#l440"></a>
<span id="l441"></span><a href="#l441"></a>
<span id="l442"><span class="c"># See http://mathworld.wolfram.com/Variance.html</span></span><a href="#l442"></a>
<span id="l443"><span class="c">#     http://mathworld.wolfram.com/SampleVariance.html</span></span><a href="#l443"></a>
<span id="l444"><span class="c">#     http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance</span></span><a href="#l444"></a>
<span id="l445"><span class="c">#</span></span><a href="#l445"></a>
<span id="l446"><span class="c"># Under no circumstances use the so-called &quot;computational formula for</span></span><a href="#l446"></a>
<span id="l447"><span class="c"># variance&quot;, as that is only suitable for hand calculations with a small</span></span><a href="#l447"></a>
<span id="l448"><span class="c"># amount of low-precision data. It has terrible numeric properties.</span></span><a href="#l448"></a>
<span id="l449"><span class="c">#</span></span><a href="#l449"></a>
<span id="l450"><span class="c"># See a comparison of three computational methods here:</span></span><a href="#l450"></a>
<span id="l451"><span class="c"># http://www.johndcook.com/blog/2008/09/26/comparing-three-methods-of-computing-standard-deviation/</span></span><a href="#l451"></a>
<span id="l452"></span><a href="#l452"></a>
<span id="l453"><span class="k">def</span> <span class="nf">_ss</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">c</span><span class="o">=</span><span class="bp">None</span><span class="p">):</span></span><a href="#l453"></a>
<span id="l454">    <span class="sd">&quot;&quot;&quot;Return sum of square deviations of sequence data.</span></span><a href="#l454"></a>
<span id="l455"></span><a href="#l455"></a>
<span id="l456"><span class="sd">    If ``c`` is None, the mean is calculated in one pass, and the deviations</span></span><a href="#l456"></a>
<span id="l457"><span class="sd">    from the mean are calculated in a second pass. Otherwise, deviations are</span></span><a href="#l457"></a>
<span id="l458"><span class="sd">    calculated from ``c`` as given. Use the second case with care, as it can</span></span><a href="#l458"></a>
<span id="l459"><span class="sd">    lead to garbage results.</span></span><a href="#l459"></a>
<span id="l460"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l460"></a>
<span id="l461">    <span class="k">if</span> <span class="n">c</span> <span class="ow">is</span> <span class="bp">None</span><span class="p">:</span></span><a href="#l461"></a>
<span id="l462">        <span class="n">c</span> <span class="o">=</span> <span class="n">mean</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l462"></a>
<span id="l463">    <span class="n">ss</span> <span class="o">=</span> <span class="n">_sum</span><span class="p">((</span><span class="n">x</span><span class="o">-</span><span class="n">c</span><span class="p">)</span><span class="o">**</span><span class="mi">2</span> <span class="k">for</span> <span class="n">x</span> <span class="ow">in</span> <span class="n">data</span><span class="p">)</span></span><a href="#l463"></a>
<span id="l464">    <span class="c"># The following sum should mathematically equal zero, but due to rounding</span></span><a href="#l464"></a>
<span id="l465">    <span class="c"># error may not.</span></span><a href="#l465"></a>
<span id="l466">    <span class="n">ss</span> <span class="o">-=</span> <span class="n">_sum</span><span class="p">((</span><span class="n">x</span><span class="o">-</span><span class="n">c</span><span class="p">)</span> <span class="k">for</span> <span class="n">x</span> <span class="ow">in</span> <span class="n">data</span><span class="p">)</span><span class="o">**</span><span class="mi">2</span><span class="o">/</span><span class="nb">len</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l466"></a>
<span id="l467">    <span class="k">assert</span> <span class="ow">not</span> <span class="n">ss</span> <span class="o">&lt;</span> <span class="mi">0</span><span class="p">,</span> <span class="s">&#39;negative sum of square deviations: </span><span class="si">%f</span><span class="s">&#39;</span> <span class="o">%</span> <span class="n">ss</span></span><a href="#l467"></a>
<span id="l468">    <span class="k">return</span> <span class="n">ss</span></span><a href="#l468"></a>
<span id="l469"></span><a href="#l469"></a>
<span id="l470"></span><a href="#l470"></a>
<span id="l471"><span class="k">def</span> <span class="nf">variance</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">xbar</span><span class="o">=</span><span class="bp">None</span><span class="p">):</span></span><a href="#l471"></a>
<span id="l472">    <span class="sd">&quot;&quot;&quot;Return the sample variance of data.</span></span><a href="#l472"></a>
<span id="l473"></span><a href="#l473"></a>
<span id="l474"><span class="sd">    data should be an iterable of Real-valued numbers, with at least two</span></span><a href="#l474"></a>
<span id="l475"><span class="sd">    values. The optional argument xbar, if given, should be the mean of</span></span><a href="#l475"></a>
<span id="l476"><span class="sd">    the data. If it is missing or None, the mean is automatically calculated.</span></span><a href="#l476"></a>
<span id="l477"></span><a href="#l477"></a>
<span id="l478"><span class="sd">    Use this function when your data is a sample from a population. To</span></span><a href="#l478"></a>
<span id="l479"><span class="sd">    calculate the variance from the entire population, see ``pvariance``.</span></span><a href="#l479"></a>
<span id="l480"></span><a href="#l480"></a>
<span id="l481"><span class="sd">    Examples:</span></span><a href="#l481"></a>
<span id="l482"></span><a href="#l482"></a>
<span id="l483"><span class="sd">    &gt;&gt;&gt; data = [2.75, 1.75, 1.25, 0.25, 0.5, 1.25, 3.5]</span></span><a href="#l483"></a>
<span id="l484"><span class="sd">    &gt;&gt;&gt; variance(data)</span></span><a href="#l484"></a>
<span id="l485"><span class="sd">    1.3720238095238095</span></span><a href="#l485"></a>
<span id="l486"></span><a href="#l486"></a>
<span id="l487"><span class="sd">    If you have already calculated the mean of your data, you can pass it as</span></span><a href="#l487"></a>
<span id="l488"><span class="sd">    the optional second argument ``xbar`` to avoid recalculating it:</span></span><a href="#l488"></a>
<span id="l489"></span><a href="#l489"></a>
<span id="l490"><span class="sd">    &gt;&gt;&gt; m = mean(data)</span></span><a href="#l490"></a>
<span id="l491"><span class="sd">    &gt;&gt;&gt; variance(data, m)</span></span><a href="#l491"></a>
<span id="l492"><span class="sd">    1.3720238095238095</span></span><a href="#l492"></a>
<span id="l493"></span><a href="#l493"></a>
<span id="l494"><span class="sd">    This function does not check that ``xbar`` is actually the mean of</span></span><a href="#l494"></a>
<span id="l495"><span class="sd">    ``data``. Giving arbitrary values for ``xbar`` may lead to invalid or</span></span><a href="#l495"></a>
<span id="l496"><span class="sd">    impossible results.</span></span><a href="#l496"></a>
<span id="l497"></span><a href="#l497"></a>
<span id="l498"><span class="sd">    Decimals and Fractions are supported:</span></span><a href="#l498"></a>
<span id="l499"></span><a href="#l499"></a>
<span id="l500"><span class="sd">    &gt;&gt;&gt; from decimal import Decimal as D</span></span><a href="#l500"></a>
<span id="l501"><span class="sd">    &gt;&gt;&gt; variance([D(&quot;27.5&quot;), D(&quot;30.25&quot;), D(&quot;30.25&quot;), D(&quot;34.5&quot;), D(&quot;41.75&quot;)])</span></span><a href="#l501"></a>
<span id="l502"><span class="sd">    Decimal(&#39;31.01875&#39;)</span></span><a href="#l502"></a>
<span id="l503"></span><a href="#l503"></a>
<span id="l504"><span class="sd">    &gt;&gt;&gt; from fractions import Fraction as F</span></span><a href="#l504"></a>
<span id="l505"><span class="sd">    &gt;&gt;&gt; variance([F(1, 6), F(1, 2), F(5, 3)])</span></span><a href="#l505"></a>
<span id="l506"><span class="sd">    Fraction(67, 108)</span></span><a href="#l506"></a>
<span id="l507"></span><a href="#l507"></a>
<span id="l508"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l508"></a>
<span id="l509">    <span class="k">if</span> <span class="nb">iter</span><span class="p">(</span><span class="n">data</span><span class="p">)</span> <span class="ow">is</span> <span class="n">data</span><span class="p">:</span></span><a href="#l509"></a>
<span id="l510">        <span class="n">data</span> <span class="o">=</span> <span class="nb">list</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l510"></a>
<span id="l511">    <span class="n">n</span> <span class="o">=</span> <span class="nb">len</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l511"></a>
<span id="l512">    <span class="k">if</span> <span class="n">n</span> <span class="o">&lt;</span> <span class="mi">2</span><span class="p">:</span></span><a href="#l512"></a>
<span id="l513">        <span class="k">raise</span> <span class="n">StatisticsError</span><span class="p">(</span><span class="s">&#39;variance requires at least two data points&#39;</span><span class="p">)</span></span><a href="#l513"></a>
<span id="l514">    <span class="n">ss</span> <span class="o">=</span> <span class="n">_ss</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">xbar</span><span class="p">)</span></span><a href="#l514"></a>
<span id="l515">    <span class="k">return</span> <span class="n">ss</span><span class="o">/</span><span class="p">(</span><span class="n">n</span><span class="o">-</span><span class="mi">1</span><span class="p">)</span></span><a href="#l515"></a>
<span id="l516"></span><a href="#l516"></a>
<span id="l517"></span><a href="#l517"></a>
<span id="l518"><span class="k">def</span> <span class="nf">pvariance</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">mu</span><span class="o">=</span><span class="bp">None</span><span class="p">):</span></span><a href="#l518"></a>
<span id="l519">    <span class="sd">&quot;&quot;&quot;Return the population variance of ``data``.</span></span><a href="#l519"></a>
<span id="l520"></span><a href="#l520"></a>
<span id="l521"><span class="sd">    data should be an iterable of Real-valued numbers, with at least one</span></span><a href="#l521"></a>
<span id="l522"><span class="sd">    value. The optional argument mu, if given, should be the mean of</span></span><a href="#l522"></a>
<span id="l523"><span class="sd">    the data. If it is missing or None, the mean is automatically calculated.</span></span><a href="#l523"></a>
<span id="l524"></span><a href="#l524"></a>
<span id="l525"><span class="sd">    Use this function to calculate the variance from the entire population.</span></span><a href="#l525"></a>
<span id="l526"><span class="sd">    To estimate the variance from a sample, the ``variance`` function is</span></span><a href="#l526"></a>
<span id="l527"><span class="sd">    usually a better choice.</span></span><a href="#l527"></a>
<span id="l528"></span><a href="#l528"></a>
<span id="l529"><span class="sd">    Examples:</span></span><a href="#l529"></a>
<span id="l530"></span><a href="#l530"></a>
<span id="l531"><span class="sd">    &gt;&gt;&gt; data = [0.0, 0.25, 0.25, 1.25, 1.5, 1.75, 2.75, 3.25]</span></span><a href="#l531"></a>
<span id="l532"><span class="sd">    &gt;&gt;&gt; pvariance(data)</span></span><a href="#l532"></a>
<span id="l533"><span class="sd">    1.25</span></span><a href="#l533"></a>
<span id="l534"></span><a href="#l534"></a>
<span id="l535"><span class="sd">    If you have already calculated the mean of the data, you can pass it as</span></span><a href="#l535"></a>
<span id="l536"><span class="sd">    the optional second argument to avoid recalculating it:</span></span><a href="#l536"></a>
<span id="l537"></span><a href="#l537"></a>
<span id="l538"><span class="sd">    &gt;&gt;&gt; mu = mean(data)</span></span><a href="#l538"></a>
<span id="l539"><span class="sd">    &gt;&gt;&gt; pvariance(data, mu)</span></span><a href="#l539"></a>
<span id="l540"><span class="sd">    1.25</span></span><a href="#l540"></a>
<span id="l541"></span><a href="#l541"></a>
<span id="l542"><span class="sd">    This function does not check that ``mu`` is actually the mean of ``data``.</span></span><a href="#l542"></a>
<span id="l543"><span class="sd">    Giving arbitrary values for ``mu`` may lead to invalid or impossible</span></span><a href="#l543"></a>
<span id="l544"><span class="sd">    results.</span></span><a href="#l544"></a>
<span id="l545"></span><a href="#l545"></a>
<span id="l546"><span class="sd">    Decimals and Fractions are supported:</span></span><a href="#l546"></a>
<span id="l547"></span><a href="#l547"></a>
<span id="l548"><span class="sd">    &gt;&gt;&gt; from decimal import Decimal as D</span></span><a href="#l548"></a>
<span id="l549"><span class="sd">    &gt;&gt;&gt; pvariance([D(&quot;27.5&quot;), D(&quot;30.25&quot;), D(&quot;30.25&quot;), D(&quot;34.5&quot;), D(&quot;41.75&quot;)])</span></span><a href="#l549"></a>
<span id="l550"><span class="sd">    Decimal(&#39;24.815&#39;)</span></span><a href="#l550"></a>
<span id="l551"></span><a href="#l551"></a>
<span id="l552"><span class="sd">    &gt;&gt;&gt; from fractions import Fraction as F</span></span><a href="#l552"></a>
<span id="l553"><span class="sd">    &gt;&gt;&gt; pvariance([F(1, 4), F(5, 4), F(1, 2)])</span></span><a href="#l553"></a>
<span id="l554"><span class="sd">    Fraction(13, 72)</span></span><a href="#l554"></a>
<span id="l555"></span><a href="#l555"></a>
<span id="l556"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l556"></a>
<span id="l557">    <span class="k">if</span> <span class="nb">iter</span><span class="p">(</span><span class="n">data</span><span class="p">)</span> <span class="ow">is</span> <span class="n">data</span><span class="p">:</span></span><a href="#l557"></a>
<span id="l558">        <span class="n">data</span> <span class="o">=</span> <span class="nb">list</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l558"></a>
<span id="l559">    <span class="n">n</span> <span class="o">=</span> <span class="nb">len</span><span class="p">(</span><span class="n">data</span><span class="p">)</span></span><a href="#l559"></a>
<span id="l560">    <span class="k">if</span> <span class="n">n</span> <span class="o">&lt;</span> <span class="mi">1</span><span class="p">:</span></span><a href="#l560"></a>
<span id="l561">        <span class="k">raise</span> <span class="n">StatisticsError</span><span class="p">(</span><span class="s">&#39;pvariance requires at least one data point&#39;</span><span class="p">)</span></span><a href="#l561"></a>
<span id="l562">    <span class="n">ss</span> <span class="o">=</span> <span class="n">_ss</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">mu</span><span class="p">)</span></span><a href="#l562"></a>
<span id="l563">    <span class="k">return</span> <span class="n">ss</span><span class="o">/</span><span class="n">n</span></span><a href="#l563"></a>
<span id="l564"></span><a href="#l564"></a>
<span id="l565"></span><a href="#l565"></a>
<span id="l566"><span class="k">def</span> <span class="nf">stdev</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">xbar</span><span class="o">=</span><span class="bp">None</span><span class="p">):</span></span><a href="#l566"></a>
<span id="l567">    <span class="sd">&quot;&quot;&quot;Return the square root of the sample variance.</span></span><a href="#l567"></a>
<span id="l568"></span><a href="#l568"></a>
<span id="l569"><span class="sd">    See ``variance`` for arguments and other details.</span></span><a href="#l569"></a>
<span id="l570"></span><a href="#l570"></a>
<span id="l571"><span class="sd">    &gt;&gt;&gt; stdev([1.5, 2.5, 2.5, 2.75, 3.25, 4.75])</span></span><a href="#l571"></a>
<span id="l572"><span class="sd">    1.0810874155219827</span></span><a href="#l572"></a>
<span id="l573"></span><a href="#l573"></a>
<span id="l574"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l574"></a>
<span id="l575">    <span class="n">var</span> <span class="o">=</span> <span class="n">variance</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">xbar</span><span class="p">)</span></span><a href="#l575"></a>
<span id="l576">    <span class="k">try</span><span class="p">:</span></span><a href="#l576"></a>
<span id="l577">        <span class="k">return</span> <span class="n">var</span><span class="o">.</span><span class="n">sqrt</span><span class="p">()</span></span><a href="#l577"></a>
<span id="l578">    <span class="k">except</span> <span class="ne">AttributeError</span><span class="p">:</span></span><a href="#l578"></a>
<span id="l579">        <span class="k">return</span> <span class="n">math</span><span class="o">.</span><span class="n">sqrt</span><span class="p">(</span><span class="n">var</span><span class="p">)</span></span><a href="#l579"></a>
<span id="l580"></span><a href="#l580"></a>
<span id="l581"></span><a href="#l581"></a>
<span id="l582"><span class="k">def</span> <span class="nf">pstdev</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">mu</span><span class="o">=</span><span class="bp">None</span><span class="p">):</span></span><a href="#l582"></a>
<span id="l583">    <span class="sd">&quot;&quot;&quot;Return the square root of the population variance.</span></span><a href="#l583"></a>
<span id="l584"></span><a href="#l584"></a>
<span id="l585"><span class="sd">    See ``pvariance`` for arguments and other details.</span></span><a href="#l585"></a>
<span id="l586"></span><a href="#l586"></a>
<span id="l587"><span class="sd">    &gt;&gt;&gt; pstdev([1.5, 2.5, 2.5, 2.75, 3.25, 4.75])</span></span><a href="#l587"></a>
<span id="l588"><span class="sd">    0.986893273527251</span></span><a href="#l588"></a>
<span id="l589"></span><a href="#l589"></a>
<span id="l590"><span class="sd">    &quot;&quot;&quot;</span></span><a href="#l590"></a>
<span id="l591">    <span class="n">var</span> <span class="o">=</span> <span class="n">pvariance</span><span class="p">(</span><span class="n">data</span><span class="p">,</span> <span class="n">mu</span><span class="p">)</span></span><a href="#l591"></a>
<span id="l592">    <span class="k">try</span><span class="p">:</span></span><a href="#l592"></a>
<span id="l593">        <span class="k">return</span> <span class="n">var</span><span class="o">.</span><span class="n">sqrt</span><span class="p">()</span></span><a href="#l593"></a>
<span id="l594">    <span class="k">except</span> <span class="ne">AttributeError</span><span class="p">:</span></span><a href="#l594"></a>
<span id="l595">        <span class="k">return</span> <span class="n">math</span><span class="o">.</span><span class="n">sqrt</span><span class="p">(</span><span class="n">var</span><span class="p">)</span></span><a href="#l595"></a></pre>
<div class="sourcelast"></div>
</div>
</div>
</div>

<script type="text/javascript">process_dates()</script>


</body>
</html>

