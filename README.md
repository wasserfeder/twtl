<h3>Overview</h3>
<strong>T</strong>ime <strong>W</strong>indow <strong>T</strong>emporal <strong>L</strong>ogic (TWTL) is a bounded temporal logic used to specify rich properties<a href="#1">[1]</a>. Relaxed versions of TWTL formulae are also considered in the sense of extending the deadlines of time windows. An automata based approach is proposed to solve synthesis, verification and learning problems. The key ingredient is a construction algorithm of annotated Deterministic Finite State Automata (DFA) from TWTL properties. See <a href="#1">[1]</a> for more details.

PyTWTL is a Python implementation of the algorithms proposed in <a href="#1">[1]</a> based on LOMAP <a href="#2">[2]</a>, ANTLRv4 <a href="#3">[3]</a> and networkx <a href="#4">[4]</a> libraries. PyTWTL implementation is released under the GPLv3 license.
The library can be used to:
<ul type="square">
    <li>construct DFAs and annotated DFAs from TWTL formulae;</li>
    <li>monitor the satisfaction of a TWTL formula;</li>
    <li>monitor the satisfaction of an arbitrary relaxation of a TWTL formula;</li>
    <li>compute the temporal relaxation of a trace with respect to a TWTL formula;</li>
    <li>compute a satisfying control policy with respect to a TWTL formula;</li>
    <li>compute a minimally relaxed control policy with respect to a TWTL formula;</li>
    <li>verify if all traces of a system satisfy some relaxed version of a TWTL formula;</li>
    <li>learn the parameters of a TWTL formula, i.e. the deadlines.</li>
</ul>

The parsing of TWTL formulae is performed using ANTLRv4 framework. The package provides a grammar file which may be used to generate lexers and parsers for other programming languages such as Java, C/C++, Ruby.

<h3>Citation</h3>
If you use TWTL or PyTWTL, then please consider citing the reference paper:

Cristian-Ioan Vasile, Derya Aksaray, and Calin Belta. <em>"Time Window Temporal Logic"</em>. Theoretical Computer Science, 691(Supplement C):27–54, August 2017. <a href="https://doi.org/10.1016/j.tcs.2017.07.012" target="_blank">doi:10.1016/j.tcs.2017.07.012</a>,
<a href="https://cristianvasile.com/sites/default/files/articole/TCS_2017.bib" target="_blank"><strong>[bib]</strong></a>

<h3>Download</h3>
<a href="https://github.com/wasserfeder/twtl.git" target="_blank">Download</a>
<br/>or<br/>
<code>git clone git@github.com:wasserfeder/twtl.git</code>

<h3>Requirements</h3>
The package is written for python 2.7. The following python packages are required:
<ul type="square">
    <li>NumPy</li>
    <li>NetworkX (<=1.11)</li>
    <li>matplotlib</li>
    <li>setuptools</li>
    <li>ANTLRv4 python runtime</li>
</ul>
You can install the packages using:<br/>
<code>pip install networkx==1.11, numpy, matplotlib, antlr4-python2-runtime==4.7.1, setuptools</code>

Also download the <a href="https://www.antlr.org/download/antlr-4.7.1-complete.jar" target="_blank">ANTLR4</a> library and set it up using
<code><pre>
cd <download_directory>
wget 'https://www.antlr.org/download/antlr-4.7.1-complete.jar'
echo "export CLASSPATH=\".:$PWD/lib/antlr-4.7.1-complete.jar:$CLASSPATH\"" >> ~/.bashrc
echo "alias antlr4=\"java -jar $PWD/lib/antlr-4.7.1-complete.jar -visitor\"" >> ~/.bashrc
echo "alias grun=\"java org.antlr.v4.gui.TestRig\"" >> ~/.bashrc
</pre></code>


<h3>How to Use</h3>
See <code>examples_tcs.py</code> for examples of the algorithms and the PyTWTL API.
An ANT build file <code>build.xml</code> is provided to generate the lexer and parser from the ANTLR4 grammar files.

<h3>License & Copying</h3>
<pre>Copyright (C) 2015-2020  Cristian Ioan Vasile &lt;cvasile@lehigh.edu&gt;<br/>
Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics Lab
Lehigh University

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

TWTL uses the ANTLR (version 4) third party Java library to generate the lexers and parsers.
See the file names 'license.txt' for copyright notices and license information of packages used in PyTWTL.

<h3>References</h3>
<p id="1">[1] Cristian Ioan Vasile, Derya Aksaray, and Calin Belta. <em>"Time Window Temporal Logic"</em>. Theoretical Computer Science, 691(Supplement C):27–54, August 2017. <a href="https://doi.org/10.1016/j.tcs.2017.07.012" target="_blank">doi:10.1016/j.tcs.2017.07.012</a>.</p>
<p id="2">[2] Alphan Ulusoy, Stephen L. Smith, Xu Chu Ding, Calin Belta, and Daniela Rus. <em>"Optimality and robustness in multi-robot path planning with temporal logic constraints."</em> The International Journal of Robotics Research 32, no. 8 (2013): 889-911.</p>
<p id="3">[3] Terence Parr. <em>"The Definitive ANTLR Reference: Building Domain-Specific Languages."</em> Pragmatic Bookshelf, 2007. ISBN 978-0978739256.</p>
<p id="4">[4] Aric A. Hagberg, Daniel A. Schult, and Pieter J. Swart. <em>"Exploring network structure, dynamics, and function using NetworkX."</em> 2008.</p>
<!-- p id="5">[5] </p>
<p id="6">[6] </p -->
