<h3>Overview</h3>
<strong>T</strong>ime <strong>W</strong>indow <strong>T</strong>emporal <strong>L</strong>ogic (TWTL) is a bounded temporal logic used to specify rich properties<a href="#1">[1]</a>. Relaxed versions of TWTL formulae are also considered in the sense of extending the deadlines of time windows. An automata based approach is proposed to solve synthesis, verification and learning problems. The key ingredient is a construction algorithm of annotated Deterministic Finite State Automata (DFA) from TWTL properties. See <a href="#1">[1]</a> for more details. 

PyTWTL is a Python 2.7 implementation of the algorithms proposed in <a href="#1">[1]</a> based on LOMAP <a href="#2">[2]</a>, ANTLRv3 <a href="#3">[3]</a> and networkx <a href="#4">[4]</a> libraries. PyTWTL implementation is released under the GPLv3 license.
The library can be used to:
<ul type="square">
<!-- li><code>todo</code> </li -->
    <li>construct DFAs and annotated DFAs from TWTL formulae;</li>
    <li>monitor the satisfaction of a TWTL formula;</li>
    <li>monitor the satisfaction of an arbitrary relaxation of a TWTL formula;</li>
    <li>compute the temporal relaxation of a trace with respect to a TWTL formula;</li>
    <li>compute a satisfying control policy with respect to a TWTL formula;</li>
    <li>compute a minimally relaxed control policy with respect to a TWTL formula;</li>
    <li>verify if all traces of a system satisfy some relaxed version of a TWTL formula;</li>
    <li>learn the parameters of a TWTL formula, i.e. the deadlines.</li>
</ul>

The parsing of TWTL formulae is performed using ANTLRv3 framework. The package provides grammar files which may be used to generate lexers and parsers for other programming languages such as Java, C/C++, Ruby. To support Python 2.7, we used version 3.1.3 of ANTLRv3 and the corresponding Python runtime ANTLR library, which we included in our distribution for convenience.

<h3>Citation</h3>
If you use TWTL or PyTWTL, then please consider citing the reference paper:

Cristian-Ioan Vasile, Derya Aksaray, and Calin Belta. <em>"Time Window Temporal Logic"</em>, arXiv preprint, <a href="http://arxiv.org/abs/1602.04294" target="_blank">arXiv:1602.04294</a>, 2016.
<a href="/hyness/files/2016/02/twtl.txt" target="_blank"><strong>[bib]</strong></a>

<h3>Download</h3>
<a href="/hyness/files/2016/02/pytwtl.zip">Download</a>

<h3>Requirements</h3>
The package is written for python 2.7. The following python packages are required:
<ul type="square">
    <li>NumPy</li>
    <li>NetworkX</li>
    <li>ParallelPython</li>
    <li>matplotlib</li>
    <li>setuptools</li>
    <li>ANTLRv3 python runtime</li>
</ul>
You can install the packages using:
<code>pip install networkx, numpy, matplotlib, pp, antlr-python-runtime, setuptools</code>

<h3>How to Use</h3>
See <code>examples_tcs.py</code> for examples of the algorithms and the PyTWTL API.
An ANT build file <code>build.xml</code> is provided to generate the lexer and parser from the ANTLR3 grammar files.

<h3>License & Copying</h3>
<pre>Copyright (C) 2015-2016  Cristian Ioan Vasile <cvasile@bu.edu>
Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab,
Boston University

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see &lt;http://www.gnu.org/licenses/&gt;.
</pre>

A copy of the GNU General Public License is included in the source folder in a file called 'gpl.txt'.

TWTL uses the ANTLR (version 3.1.3) third party Java library to generate the lexers and parsers. It is included for convenience in the 'lib' folder of the distribution source tree. The library can be downloaded from <a href="https://github.com/antlr/website-antlr3/tree/gh-pages/download" target="_blank">https://github.com/antlr/website-antlr3/tree/gh-pages/download</a>.
See the file names 'license.txt' for copyright notices and license information of packages used in PyTWTL.

<h3>References</h3>
<p id="1">[1] Cristian-Ioan Vasile, Derya Aksaray, and Calin Belta. <em>"Time Window Temporal Logic."</em> arXiv preprint, <a href="http://arxiv.org/abs/1602.04294" target="_blank">arXiv:1602.04294</a>, 2016.</p>
<p id="2">[2] Alphan Ulusoy, Stephen L. Smith, Xu Chu Ding, Calin Belta, and Daniela Rus. <em>"Optimality and robustness in multi-robot path planning with temporal logic constraints."</em> The International Journal of Robotics Research 32, no. 8 (2013): 889-911.</p>
<p id="3">[3] Terence Parr. <em>"The Definitive ANTLR Reference: Building Domain-Specific Languages."</em> Pragmatic Bookshelf, 2007. ISBN 978-0978739256.</p>
<p id="4">[4] Aric A. Hagberg, Daniel A. Schult, and Pieter J. Swart. <em>"Exploring network structure, dynamics, and function using NetworkX."</em> 2008.</p>
<!-- p id="5">[5] </p>
<p id="6">[6] </p -->